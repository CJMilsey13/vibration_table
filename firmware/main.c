/*
 * ICM-42688-P → RP2350 dual-core → USB CDC streamer
 * ==================================================
 * Core 0  Initialises IMU over SPI, then reads exactly one sample per sensor
 *         data-ready (DRDY polled over SPI in INT_STATUS — the INT1 pin is
 *         NOT used or needed) and pushes raw int16 triplets, each stamped
 *         with a sequence number, into a lock-free SPSC ring buffer.
 * Core 1  Drains the ring buffer; formats 10-byte binary frames;
 *         writes them to the host over USB CDC (stdio_usb).
 *
 * Wire protocol (little-endian):
 *   [0xAA][0x55] | seq uint16 | ax int16 | ay int16 | az int16   = 10 bytes
 *   Every BANNER_EVERY frames one ASCII line starting with '#' is sent
 *   between frames: firmware id, the sensor's filter registers, and odr= the
 *   sensor's measured sample rate in Hz. Hosts that do not want it skip it by
 *   synchronising on 0xAA 0x55, as they already must.
 *
 * The frame rate is the SENSOR's output data rate — nominally 8 kHz, but set
 * by the sensor's own oscillator and about 1.3 % fast on the unit this was
 * developed with (8108 Hz). The host resamples to exactly 8000 Hz using odr=.
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
#define ICM_REG_INT_STATUS      0x2Du   /* bit 3 = UI_DRDY_INT, cleared by reading */
#define ICM_INT_STATUS_DRDY     0x08u
#define ICM_REG_INTF_CONFIG1    0x4Du   /* [1:0] CLKSEL; default 0x91 */
#define ICM_REG_PWR_MGMT0       0x4Eu
#define ICM_REG_GYRO_CONFIG0    0x4Fu   /* [7:5] GYRO_FS_SEL  [3:0] GYRO_ODR */
#define ICM_REG_ACCEL_CONFIG0   0x50u
#define ICM_REG_GYRO_ACCEL_CONFIG0  0x52u   /* [7:4] ACCEL_UI_FILT_BW  [3:0] GYRO_UI_FILT_BW */
#define ICM_REG_ACCEL_CONFIG1   0x53u   /* [4:3] ACCEL_UI_FILT_ORD */
#define ICM_REG_INT_CONFIG      0x14u
#define ICM_REG_INT_SOURCE0     0x65u
#define ICM_REG_WHO_AM_I        0x75u
#define ICM_WHO_AM_I_EXPECTED   0x47u

/* ── ICM-42688-P register map (bank 2) — accel anti-alias filter ───────────── */
#define ICM_REG_ACCEL_CONFIG_STATIC2  0x03u   /* [6:1] ACCEL_AAF_DELT  [0] ACCEL_AAF_DIS */
#define ICM_REG_ACCEL_CONFIG_STATIC3  0x04u   /* ACCEL_AAF_DELTSQR[7:0] */
#define ICM_REG_ACCEL_CONFIG_STATIC4  0x05u   /* [7:4] ACCEL_AAF_BITSHIFT  [3:0] ACCEL_AAF_DELTSQR[11:8] */

/*
 * Accelerometer filters.
 *
 * Out of reset the accel path is band-limited well inside the 20–2000 Hz test
 * band: a 2nd-order anti-alias filter (AAF) near 1.2 kHz followed by a UI
 * filter at ODR/4 = 2 kHz. Measured on this rig from the at-rest noise floor,
 * the response was −4.6 dB at 1.0–1.4 kHz and −9 dB at 1.4–2.0 kHz. A control
 * loop reading that as the rig's response over-drives the top of the band by
 * the same amount.
 *
 * So both are opened as far as they go:
 *   AAF        DELT 63, DELTSQR 3968, BITSHIFT 3  → 3979 Hz (the widest setting)
 *   UI filter  ACCEL_UI_FILT_BW = 0               → ODR/2 = 4 kHz
 * The gyro's UI filter nibble is left at its reset value (1).
 *
 * These live in bank 2 / are static configuration: they are written while the
 * sensors are still OFF, i.e. before PWR_MGMT0.
 */
#define ICM_ACCEL_AAF_DELT       63u
#define ICM_ACCEL_AAF_DELTSQR    3968u
#define ICM_ACCEL_AAF_BITSHIFT   3u
#define ICM_ACCEL_STATIC2_VALUE  ((uint8_t)(ICM_ACCEL_AAF_DELT << 1))                /* AAF enabled */
#define ICM_ACCEL_STATIC3_VALUE  ((uint8_t)(ICM_ACCEL_AAF_DELTSQR & 0xFFu))
#define ICM_ACCEL_STATIC4_VALUE  ((uint8_t)((ICM_ACCEL_AAF_BITSHIFT << 4) | (ICM_ACCEL_AAF_DELTSQR >> 8)))
#define ICM_GYRO_ACCEL_CONFIG0_VALUE  0x01u   /* accel UI BW = ODR/2, gyro UI BW = reset default */
#define ICM_ACCEL_CONFIG1_VALUE       0x0Du   /* reset value: 2nd-order UI filter */

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
#define ICM_ACCEL_ODR_8K        0x03u

#define ICM_ACCEL_CONFIG        (ICM_ACCEL_FS_SEL_16G | ICM_ACCEL_ODR_8K)

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
 * INT1 is not used: sampling is paced by polling UI_DRDY in INT_STATUS, which
 * needs no extra wire. The two definitions below are kept for a future
 * interrupt-driven build only.
 *
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
#define FW_VERSION  "2026-10-08"
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
 * 4096 entries × 8 bytes = 32 kB ≈ 512 ms headroom at 8 kHz.
 */
#define RING_SIZE  4096u

typedef struct { int16_t ax, ay, az; uint16_t seq; } sample_t;

static volatile sample_t ring_buf[RING_SIZE];
static volatile uint32_t ring_wr = 0u;   /* written only by Core 0 */
static volatile uint32_t ring_rd = 0u;   /* written only by Core 1 */

/*
 * Acquisition counter — Core 0 only. It advances for every sample READ from
 * the IMU, whether or not that sample fits in the ring, and travels with the
 * sample into the frame. A sample dropped on overrun therefore leaves a gap in
 * the sequence the host sees, and the host's drop counter reports it.
 * Numbering frames at transmit time instead would hide every overrun.
 */
static uint16_t acq_seq = 0u;

/*
 * The sensor's output data rate in milli-hertz, measured on Core 0 against
 * this MCU's crystal by counting samples over a few seconds. 0 until the
 * first measurement is in (about 1 s after boot). Read by Core 1 for the
 * banner; a single aligned 32-bit word, so no lock is needed.
 */
static volatile uint32_t odr_mhz = 0u;

/* Samples the sensor produced that Core 0 never read. They get sequence
 * numbers too, so the host counts them as drops like any other lost sample. */
static inline void ring_skip(uint32_t missed)
{
    acq_seq = (uint16_t)(acq_seq + missed);
}

static inline bool ring_full(void)
{
    return (ring_wr - ring_rd) >= RING_SIZE;
}

static inline void ring_push(int16_t ax, int16_t ay, int16_t az)
{
    uint16_t seq = acq_seq++;
    if (ring_full()) return;   /* overrun — drop sample rather than block IRQ */
    uint32_t idx = ring_wr & (RING_SIZE - 1u);
    ring_buf[idx].ax  = ax;
    ring_buf[idx].ay  = ay;
    ring_buf[idx].az  = az;
    ring_buf[idx].seq = seq;
    __dmb();          /* payload visible before head advances */
    ring_wr++;
}

static inline bool ring_pop(sample_t *out)
{
    if (ring_wr == ring_rd) return false;
    uint32_t idx = ring_rd & (RING_SIZE - 1u);
    out->ax  = ring_buf[idx].ax;
    out->ay  = ring_buf[idx].ay;
    out->az  = ring_buf[idx].az;
    out->seq = ring_buf[idx].seq;
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
 *   4 flashes = filter config readback   (AAF / UI filter write didn't stick)
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

/* Filter registers as found after soft reset, before this firmware changed
 * them. Reported in the banner so the host can see what the part defaults to. */
static uint8_t icm_reset_cfg[5];   /* GYRO_ACCEL_CONFIG0, ACCEL_CONFIG1, STATIC2, STATIC3, STATIC4 */

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

    /* Configure ODR + FSR BEFORE enabling — datasheet-recommended order. */
    icm_write(ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_CONFIG);   /* ±16 g, 8 kHz */
    icm_write(ICM_REG_GYRO_CONFIG0,  0x03u);              /* ±2000 dps, 8 kHz */
    sleep_ms(1);

    /* Accel filters — also BEFORE enabling: bank-2 registers must only be
     * written with accel and gyro off. Note what the part reset to first. */
    icm_reset_cfg[0] = icm_read_byte(ICM_REG_GYRO_ACCEL_CONFIG0);
    icm_reset_cfg[1] = icm_read_byte(ICM_REG_ACCEL_CONFIG1);
    icm_write(ICM_REG_REG_BANK_SEL, 0x02u);
    icm_reset_cfg[2] = icm_read_byte(ICM_REG_ACCEL_CONFIG_STATIC2);
    icm_reset_cfg[3] = icm_read_byte(ICM_REG_ACCEL_CONFIG_STATIC3);
    icm_reset_cfg[4] = icm_read_byte(ICM_REG_ACCEL_CONFIG_STATIC4);
    icm_write(ICM_REG_ACCEL_CONFIG_STATIC2, ICM_ACCEL_STATIC2_VALUE);
    icm_write(ICM_REG_ACCEL_CONFIG_STATIC3, ICM_ACCEL_STATIC3_VALUE);
    icm_write(ICM_REG_ACCEL_CONFIG_STATIC4, ICM_ACCEL_STATIC4_VALUE);
    const bool aaf_ok =
        icm_read_byte(ICM_REG_ACCEL_CONFIG_STATIC2) == ICM_ACCEL_STATIC2_VALUE &&
        icm_read_byte(ICM_REG_ACCEL_CONFIG_STATIC3) == ICM_ACCEL_STATIC3_VALUE &&
        icm_read_byte(ICM_REG_ACCEL_CONFIG_STATIC4) == ICM_ACCEL_STATIC4_VALUE;
    icm_write(ICM_REG_REG_BANK_SEL, 0x00u);               /* back to bank 0 — always */
    icm_write(ICM_REG_GYRO_ACCEL_CONFIG0, ICM_GYRO_ACCEL_CONFIG0_VALUE);
    icm_write(ICM_REG_ACCEL_CONFIG1,      ICM_ACCEL_CONFIG1_VALUE);
    if (!aaf_ok ||
        icm_read_byte(ICM_REG_GYRO_ACCEL_CONFIG0) != ICM_GYRO_ACCEL_CONFIG0_VALUE ||
        icm_read_byte(ICM_REG_ACCEL_CONFIG1)      != ICM_ACCEL_CONFIG1_VALUE)
        blink_fault(4);   /* never returns */
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
        if (icm_read_byte(ICM_REG_INT_STATUS) & ICM_INT_STATUS_DRDY) { drdy = true; break; }
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
            if (icm_read_byte(ICM_REG_INT_STATUS) & ICM_INT_STATUS_DRDY) break;
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
 * At 8 kHz, WRITE_BATCH=64 → 125 fwrite calls/s, each 640 bytes (~10 USB packets).
 */
#define WRITE_BATCH  64u

/*
 * One line of text, repeated every BANNER_EVERY frames (2 s). Says which
 * firmware this is and what the sensor's filter registers were at reset and
 * are now, in the order GYRO_ACCEL_CONFIG0, ACCEL_CONFIG1, ACCEL_CONFIG_STATIC2,
 * STATIC3, STATIC4. It is plain ASCII, so it holds no 0xAA byte and cannot be
 * mistaken for a frame.
 *
 * It is repeated rather than sent once at connection because a host opening
 * the port typically purges its receive buffer just after raising DTR, which
 * discards anything sent in the first few milliseconds.
 */
#define BANNER_EVERY  16000u
#define BANNER_SECOND  4000u   /* the second one follows the first by 0.5 s */

static void send_banner(void)
{
    const uint32_t odr = odr_mhz;
    printf("# icm42688_streamer " FW_VERSION " drdy-polled odr=%lu.%03lu"
           " reset=%02X,%02X,%02X,%02X,%02X now=%02X,%02X,%02X,%02X,%02X\n",
           (unsigned long)(odr / 1000u), (unsigned long)(odr % 1000u),
           icm_reset_cfg[0], icm_reset_cfg[1], icm_reset_cfg[2],
           icm_reset_cfg[3], icm_reset_cfg[4],
           ICM_GYRO_ACCEL_CONFIG0_VALUE, ICM_ACCEL_CONFIG1_VALUE,
           ICM_ACCEL_STATIC2_VALUE, ICM_ACCEL_STATIC3_VALUE, ICM_ACCEL_STATIC4_VALUE);
}

static void core1_main(void)
{
    /* USB CDC is initialised on Core 0 before this core is launched. */
    while (!stdio_usb_connected())
        sleep_ms(10);

    /* Core 0 has been sampling since boot, so the ring holds up to 512 ms of
     * data from before the host connected. Discard it: the host should start
     * with what the sensor is reading now. Core 1 is the only writer of
     * ring_rd, so this is safe against Core 0's concurrent pushes. */
    ring_rd = ring_wr;

    uint8_t  out[WRITE_BATCH * FRAME_BYTES];
    uint32_t out_idx = 0u;
    uint32_t since_banner = BANNER_EVERY;   /* so the first one goes out at once */
    bool     second_due   = true;           /* ...and a second soon after, in case
                                             * the host purged the first */
    sample_t s;

    while (true) {
        if (!ring_pop(&s)) {
            tight_loop_contents();
            continue;
        }

        uint8_t *f = out + out_idx * FRAME_BYTES;
        f[0] = SYNC_A;
        f[1] = SYNC_B;
        f[2] = (uint8_t)(s.seq);            /* stamped at acquisition, Core 0 */
        f[3] = (uint8_t)(s.seq >> 8);
        f[4] = (uint8_t)(s.ax);
        f[5] = (uint8_t)((uint16_t)s.ax >> 8);
        f[6] = (uint8_t)(s.ay);
        f[7] = (uint8_t)((uint16_t)s.ay >> 8);
        f[8] = (uint8_t)(s.az);
        f[9] = (uint8_t)((uint16_t)s.az >> 8);
        out_idx++;

        if (out_idx == WRITE_BATCH) {
            /* Only ever between whole frames, so it cannot split one. */
            since_banner += WRITE_BATCH;
            if (since_banner >= BANNER_EVERY) {
                send_banner();
                since_banner = second_due ? BANNER_EVERY - BANNER_SECOND : 0u;
                second_due   = false;
            }
            fwrite(out, 1u, sizeof(out), stdout);
            fflush(stdout);
            out_idx = 0u;
        }
    }
}

/* ── Core 0: SPI init + IMU sampling ───────────────────────────────────────── */
#define ODR_PERIOD_US  125u   /* nominal — the sensor's clock sets the real one */
#define DRDY_QUIET_US   90u   /* no SPI traffic for this long after each sample */
int main(void)
{
    /* USB CDC — TinyUSB task runs via USB IRQ, safe to call fwrite from Core 1 */
    stdio_usb_init();
    stdio_set_translate_crlf(&stdio_usb, false);   /* binary output — no CR/LF translation */

    spi_init(SPI_PORT, 8u * 1000u * 1000u);   /* 8 MHz — 7-byte burst = 7 µs, well within 125 µs ODR period */
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

    /*
     * Sampling loop — exactly one read per sensor sample.
     *
     * The sensor makes samples on its own 8 kHz clock, which is not locked to
     * this MCU's. Reading on an MCU timer instead (as this loop used to) lets
     * the two drift through each other: measured here, the same sample was
     * read twice about 5 times a second. So the sensor paces the loop: wait
     * for UI_DRDY in INT_STATUS, read, repeat. Reading INT_STATUS clears the
     * flag, so each sample is taken once and only once.
     *
     * DRDY cannot come sooner than one ODR period after the last, so the bus
     * is left idle for most of the period and polled only when it is due.
     *
     * The sample rate is therefore the SENSOR's 8 kHz, not the MCU's. The
     * host treats it as 8000 Hz nominal.
     */
    uint64_t last = time_us_64();
    uint64_t win_t0  = last;      /* ODR measurement window: start time, */
    uint32_t win_n   = 0u;        /* sensor samples so far,              */
    uint32_t win_len = 8192u;     /* and length — short at first, then long */
    while (true) {
        while (time_us_64() - last < DRDY_QUIET_US) tight_loop_contents();
        while (!(icm_read_byte(ICM_REG_INT_STATUS) & ICM_INT_STATUS_DRDY)) { /* poll */ }

        /* If this core was held up for two periods or more (USB servicing runs
         * here), the sensor has overwritten samples we never saw. Number them
         * so the gap is visible to the host instead of silently closing up.
         * Rounded down on purpose: a read that is merely late, by less than a
         * period, has lost nothing and must not be reported as a drop. */
        const uint64_t now = time_us_64();
        const uint32_t periods = (uint32_t)((now - last) / ODR_PERIOD_US);
        if (periods > 1u) ring_skip(periods - 1u);
        last = now;

        read_and_push();

        /* Sensor rate against this MCU's crystal: samples per elapsed time.
         * Over 32768 samples (4 s) the few µs of polling jitter at each end is
         * about 1 ppm. */
        win_n += (periods > 1u) ? periods : 1u;
        if (win_n >= win_len) {
            odr_mhz = (uint32_t)(((uint64_t)win_n * 1000000000ull) / (now - win_t0));
            win_t0  = now;
            win_n   = 0u;
            win_len = 32768u;
        }
    }
}
