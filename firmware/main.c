/*
 * ICM-42688-P → RP2350 dual-core → USB CDC streamer
 * ==================================================
 * Core 0  Initialises IMU over SPI; handles INT1 GPIO interrupt at 8 kHz;
 *         pushes raw int16 triplets into a lock-free SPSC ring buffer.
 * Core 1  Drains the ring buffer; formats 10-byte binary frames;
 *         writes them to the host over USB CDC (stdio_usb).
 *
 * Wire protocol (little-endian):
 *   [0xAA][0x55] | seq uint16 | ax int16 | ay int16 | az int16   = 10 bytes
 *
 * Default pin assignments (adjust to your board wiring):
 *   SPI0  SCK  GP18   MOSI GP19   MISO GP16   CS GP17
 *   INT1       GP20
 *
 * Build:  see CMakeLists.txt
 * SDK:    pico-sdk ≥ 2.0  (RP2350 / Pico Plus 2 support)
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"   /* __dmb() */

/* ── Pin assignments ───────────────────────────────────────────────────────── */
#define SPI_PORT    spi0
#define PIN_SCK     18u
#define PIN_MOSI    19u
#define PIN_MISO    16u
#define PIN_CS      17u
#define PIN_INT1    20u

/* ── ICM-42688-P register map (bank 0) ─────────────────────────────────────── */
#define ICM_REG_DEVICE_CONFIG   0x11u
#define ICM_REG_TEMP_DATA1      0x1Du   /* temperature MSB; LSB at 0x1E */
#define ICM_REG_REG_BANK_SEL    0x76u   /* [1:0] BANK_SEL; 0 = bank 0 (default) */
#define ICM_REG_ACCEL_DATA_X1   0x1Fu   /* first byte of 6-byte accel burst */
#define ICM_REG_INT_STATUS      0x2Du   /* bit 3 = UI_DRDY_INT */
#define ICM_REG_INTF_CONFIG1    0x4Du   /* [1:0] CLKSEL; default 0x91 */
#define ICM_REG_PWR_MGMT0       0x4Eu
#define ICM_REG_ACCEL_CONFIG0   0x50u
#define ICM_REG_INT_CONFIG      0x14u
#define ICM_REG_INT_SOURCE0     0x65u
#define ICM_REG_WHO_AM_I        0x75u
#define ICM_WHO_AM_I_EXPECTED   0x47u

/*
 * ACCEL_CONFIG0 (0x50):
 *   [7:5] ACCEL_FS_SEL  000=±16g  001=±8g  010=±4g  011=±2g
 *   [3:0] ACCEL_ODR     0110=8 kHz (Low-Noise mode)
 *
 * Change ICM_ACCEL_FS_SEL to match the FSR selected in the Python UI.
 */
#define ICM_ACCEL_FS_SEL_16G    (0x00u << 5)   /* ±16 g  — 2048 LSB/g */
#define ICM_ACCEL_FS_SEL_8G     (0x01u << 5)   /* ±8 g   — 4096 LSB/g */
#define ICM_ACCEL_FS_SEL_4G     (0x02u << 5)   /* ±4 g   — 8192 LSB/g */
#define ICM_ACCEL_FS_SEL_2G     (0x03u << 5)   /* ±2 g   — 16384 LSB/g */
#define ICM_ACCEL_ODR_1K        0x06u

#define ICM_ACCEL_CONFIG        (ICM_ACCEL_FS_SEL_16G | ICM_ACCEL_ODR_1K)

/*
 * PWR_MGMT0 (0x4E):
 *   [3:2] ACCEL_MODE  11=Low-Noise  10=Low-Power
 *   [1:0] GYRO_MODE   11=Low-Noise  00=off
 *
 * Accel LN + Gyro LN (0x0F): enabling the gyro starts the shared PLL which the
 * accel ADC requires. Accel-only modes (0x0C / 0x08) stall the ADC at 0x8000.
 * Gyro data is produced but not transmitted — only accel frames are streamed.
 */
#define ICM_PWR_ACCEL_LN_GYRO_LN  0x0Fu

/*
 * INT_CONFIG (0x14) bits [2:0] for INT1:
 *   [2] INT1_MODE          0=pulsed
 *   [1] INT1_DRIVE_CIRCUIT 1=push-pull
 *   [0] INT1_POLARITY      1=active-high
 */
#define ICM_INT1_PP_ACTIVE_HIGH  0x03u

/*
 * INT_SOURCE0 (0x65):
 *   [4] UI_DRDY_INT1_EN — route data-ready to INT1
 */
#define ICM_DRDY_INT1_EN  0x10u

/* ── Wire protocol ─────────────────────────────────────────────────────────── */
#define SYNC_A  0xAAu
#define SYNC_B  0x55u
#define FRAME_BYTES  10u   /* 2 sync + 2 seq + 6 accel */

/* ── Lock-free SPSC ring buffer ────────────────────────────────────────────── */
/*
 * Core 0 is the sole producer (ring_wr); Core 1 is the sole consumer (ring_rd).
 * A __dmb() barrier before each pointer advance ensures the other core sees the
 * payload before it sees the updated index.
 *
 * RING_SIZE must be a power of 2.
 * 4096 entries × 6 bytes = 24 kB ≈ 512 ms headroom at 8 kHz.
 */
#define RING_SIZE  4096u

typedef struct { int16_t ax, ay, az; } sample_t;

static volatile sample_t ring_buf[RING_SIZE];
static volatile uint32_t ring_wr = 0u;   /* written only by Core 0 */
static volatile uint32_t ring_rd = 0u;   /* written only by Core 1 */

static inline bool ring_full(void)
{
    return (ring_wr - ring_rd) >= RING_SIZE;
}

static inline void ring_push(int16_t ax, int16_t ay, int16_t az)
{
    if (ring_full()) return;   /* overrun — drop sample rather than block IRQ */
    uint32_t idx = ring_wr & (RING_SIZE - 1u);
    ring_buf[idx].ax = ax;
    ring_buf[idx].ay = ay;
    ring_buf[idx].az = az;
    __dmb();          /* payload visible before head advances */
    ring_wr++;
}

static inline bool ring_pop(sample_t *out)
{
    if (ring_wr == ring_rd) return false;
    uint32_t idx = ring_rd & (RING_SIZE - 1u);
    out->ax = ring_buf[idx].ax;
    out->ay = ring_buf[idx].ay;
    out->az = ring_buf[idx].az;
    __dmb();          /* payload read before tail advances */
    ring_rd++;
    return true;
}

/* ── SPI helpers ───────────────────────────────────────────────────────────── */
static inline void cs_low(void)  { gpio_put(PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_CS, 1); }

static void icm_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg & 0x7Fu, val };   /* MSB=0 → write */
    cs_low();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_high();
}

static uint8_t icm_read_byte(uint8_t reg)
{
    uint8_t tx[2] = { reg | 0x80u, 0u };    /* MSB=1 → read */
    uint8_t rx[2] = { 0u, 0u };
    cs_low();
    spi_write_read_blocking(SPI_PORT, tx, rx, 2);
    cs_high();
    return rx[1];
}

/* Burst-read 6 bytes starting at reg (accel X1..Z0) into dst[0..5]. */
static void icm_burst_read6(uint8_t reg, uint8_t *dst)
{
    uint8_t tx[7] = { reg | 0x80u, 0u, 0u, 0u, 0u, 0u, 0u };
    uint8_t rx[7];
    cs_low();
    spi_write_read_blocking(SPI_PORT, tx, rx, 7);
    cs_high();
    memcpy(dst, rx + 1, 6);
}

/* ── LED blink fault codes ──────────────────────────────────────────────────── */
/* Call to halt with a repeating blink pattern. Count the flashes per burst:
 *   2 flashes = WHO_AM_I mismatch        (wrong chip or SPI not connected)
 *   3 flashes = PWR_MGMT0 write fail     (accel enable didn't stick)
 *   5 flashes = DRDY timeout (500 ms)    (ODR timer / analog chain not started)
 *   6 flashes = DRDY ok, all data 0x8000 (accel + temp — likely MISO stuck low)
 *   7 flashes = DRDY ok, temp valid, accel 0x8000  (ADC chain not converting)
 */
static void blink_fault(uint count)
{
    const uint led = PICO_DEFAULT_LED_PIN;
    gpio_init(led);
    gpio_set_dir(led, GPIO_OUT);
    while (true) {
        for (uint i = 0; i < count; i++) {
            gpio_put(led, 1); sleep_ms(150);
            gpio_put(led, 0); sleep_ms(150);
        }
        sleep_ms(800);   /* pause between bursts */
    }
}

/* ── IMU initialisation ─────────────────────────────────────────────────────── */
static bool icm_init(void)
{
    /* Explicit bank 0 select — defensive, reset default is 0 */
    icm_write(ICM_REG_REG_BANK_SEL, 0x00u);

    /* Soft reset; datasheet requires ≥1 ms before any register access */
    icm_write(ICM_REG_DEVICE_CONFIG, 0x01u);
    sleep_ms(10);

    if (icm_read_byte(ICM_REG_WHO_AM_I) != ICM_WHO_AM_I_EXPECTED)
        blink_fault(2);   /* never returns */

    /* Explicitly write INTF_CONFIG1: PLL clock (bits[1:0]=01), wake-up osc for LP.
     * Default is 0x91; writing it ensures a clean state after any partial reset. */
    icm_write(ICM_REG_INTF_CONFIG1, 0x91u);
    sleep_ms(1);

    /* Configure ODR + FSR BEFORE enabling the accel — datasheet-recommended order.
     * NOTE: ACCEL_CONFIG0 reset default is 0x06 = ICM_ACCEL_CONFIG (±16 g / 1 kHz),
     * so readback would always pass; omitted. */
    icm_write(ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_CONFIG);
    sleep_ms(1);

    /* Enable accel + gyro LN AFTER configuring ODR/FSR.
     * Gyro LN starts the shared PLL; accel ADC requires it to convert. */
    icm_write(ICM_REG_PWR_MGMT0, ICM_PWR_ACCEL_LN_GYRO_LN);
    sleep_ms(50);   /* datasheet: up to 30 ms for analog chain; 50 ms for margin */
    if (icm_read_byte(ICM_REG_PWR_MGMT0) != ICM_PWR_ACCEL_LN_GYRO_LN)
        blink_fault(3);   /* never returns */

    /* Poll INT_STATUS for UI_DRDY (bit 3).
     * At 1 kHz ODR the first sample must arrive within ~2 ms.
     * Allow 500 ms; timeout means the ODR timer never started. */
    bool drdy = false;
    for (uint32_t ms = 0u; ms < 500u; ms++) {
        if (icm_read_byte(ICM_REG_INT_STATUS) & 0x08u) { drdy = true; break; }
        sleep_ms(1);
    }
    if (!drdy)
        blink_fault(5);   /* never returns */

    /* DRDY asserted.  Some chips take a few ODR cycles to flush the 0x8000 sentinel.
     * Check 20 consecutive DRDY pulses; pass as soon as any accel reading is valid. */
    bool accel_ok   = false;
    bool temp_valid = false;
    for (uint32_t attempt = 0u; attempt < 20u; attempt++) {
        /* Wait for the next DRDY */
        for (uint32_t ms = 0u; ms < 10u; ms++) {
            if (icm_read_byte(ICM_REG_INT_STATUS) & 0x08u) break;
            sleep_ms(1);
        }

        uint8_t raw[6];
        icm_burst_read6(ICM_REG_ACCEL_DATA_X1, raw);
        if (!(raw[0] == 0x80u && raw[1] == 0x00u &&
              raw[2] == 0x80u && raw[3] == 0x00u &&
              raw[4] == 0x80u && raw[5] == 0x00u)) {
            accel_ok = true;
        }

        const uint8_t t1 = icm_read_byte(ICM_REG_TEMP_DATA1);
        const uint8_t t0 = icm_read_byte(ICM_REG_TEMP_DATA1 + 1u);
        if (!(t1 == 0x80u && t0 == 0x00u)) temp_valid = true;

        if (accel_ok) break;
    }

    if (!accel_ok && !temp_valid)
        blink_fault(6);   /* DRDY ok, all data still 0x8000 — MISO stuck or SPI issue */
    if (!accel_ok)
        blink_fault(7);   /* DRDY ok, temp valid, accel 0x8000 — ADC not converting */

    return true;
}

/* ── Core 0 sample helper ───────────────────────────────────────────────────── */
static void read_and_push(void)
{
    uint8_t raw[6];
    icm_burst_read6(ICM_REG_ACCEL_DATA_X1, raw);

    /* ICM outputs high byte first: X1(high) X0(low) Y1 Y0 Z1 Z0 */
    int16_t ax = (int16_t)((uint16_t)raw[0] << 8 | raw[1]);
    int16_t ay = (int16_t)((uint16_t)raw[2] << 8 | raw[3]);
    int16_t az = (int16_t)((uint16_t)raw[4] << 8 | raw[5]);

    ring_push(ax, ay, az);
}

/* ── Core 1: USB CDC output ─────────────────────────────────────────────────── */
/*
 * Batches WRITE_BATCH frames into a single fwrite to reduce mutex overhead.
 * At 8 kHz, WRITE_BATCH=16 → 500 fwrite calls/s, each 160 bytes (2.5 USB packets).
 */
#define WRITE_BATCH  16u

static void core1_main(void)
{
    /* USB CDC is initialised on Core 0 before this core is launched. */
    while (!stdio_usb_connected())
        sleep_ms(10);

    uint16_t seq = 0u;
    uint8_t  out[WRITE_BATCH * FRAME_BYTES];
    uint32_t out_idx = 0u;
    sample_t s;

    while (true) {
        if (!ring_pop(&s)) {
            tight_loop_contents();
            continue;
        }

        uint8_t *f = out + out_idx * FRAME_BYTES;
        f[0] = SYNC_A;
        f[1] = SYNC_B;
        f[2] = (uint8_t)(seq);
        f[3] = (uint8_t)(seq >> 8);
        f[4] = (uint8_t)(s.ax);
        f[5] = (uint8_t)((uint16_t)s.ax >> 8);
        f[6] = (uint8_t)(s.ay);
        f[7] = (uint8_t)((uint16_t)s.ay >> 8);
        f[8] = (uint8_t)(s.az);
        f[9] = (uint8_t)((uint16_t)s.az >> 8);
        seq++;
        out_idx++;

        if (out_idx == WRITE_BATCH) {
            fwrite(out, 1u, sizeof(out), stdout);
            fflush(stdout);
            out_idx = 0u;
        }
    }
}

/* ── Core 0: SPI init + IMU sampling ───────────────────────────────────────── */
int main(void)
{
    /* USB CDC — TinyUSB task runs via USB IRQ, safe to call fwrite from Core 1 */
    stdio_usb_init();
    stdio_set_translate_crlf(&stdio_usb, false);   /* binary output — no CR/LF translation */

    /* 1 MHz — conservative for cheap breakout boards with onboard level shifters.
     * Increase toward 8 MHz once data is confirmed valid. */
    spi_init(SPI_PORT, 1u * 1000u * 1000u);
    /* ICM-42688-P supports Mode 0 and Mode 3; Mode 0 used here */
    spi_set_format(SPI_PORT, 8u, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1u);   /* deassert */

    sleep_ms(10);   /* power-on settling */

    icm_init();   /* halts with blink code on any failure, never returns false */

    multicore_launch_core1(core1_main);

    /* Polling loop — read one sample every 1000 µs (= 1 kHz).
     * SPI burst takes ~56 µs at 1 MHz, leaving ~944 µs of margin. */
    uint64_t next = time_us_64();
    while (true) {
        read_and_push();
        next += 1000u;
        while (time_us_64() < next) tight_loop_contents();
    }
}
