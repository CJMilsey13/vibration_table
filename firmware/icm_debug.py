"""
ICM-42688-P live register debug — runs on the Pico via mpremote.

Usage:
    pip install mpremote
    python -m mpremote run firmware/icm_debug.py
"""

from machine import SPI, Pin
import time

spi = SPI(0, baudrate=1_000_000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs = Pin(17, Pin.OUT, value=1)


def reg_read(reg):
    cs(0)
    buf = bytearray(2)
    spi.write_readinto(bytes([reg | 0x80, 0]), buf)
    cs(1)
    return buf[1]


def reg_write(reg, val):
    cs(0)
    spi.write(bytes([reg & 0x7F, val]))
    cs(1)


def burst_read(reg, n):
    cs(0)
    buf = bytearray(n + 1)
    spi.write_readinto(bytes([reg | 0x80] + [0] * n), buf)
    cs(1)
    return buf[1:]


# ── Reset ──────────────────────────────────────────────────────────────────────
reg_write(0x11, 0x01)   # DEVICE_CONFIG: soft reset
time.sleep_ms(10)

who = reg_read(0x75)    # WHO_AM_I
print(f"WHO_AM_I      = {who:#04x}  (expect 0x47, {'OK' if who == 0x47 else 'FAIL'})")

# ── Configure (ODR/FSR first, then enable) ─────────────────────────────────────
reg_write(0x4D, 0x91)   # INTF_CONFIG1: PLL clock, wake-up osc for LP
time.sleep_ms(1)

reg_write(0x50, 0x06)   # ACCEL_CONFIG0: ±16 g, 1 kHz ODR
time.sleep_ms(1)

reg_write(0x4E, 0x0F)   # PWR_MGMT0: accel LN + gyro LN — test both ADCs
time.sleep_ms(50)       # gyro needs longer startup than accel

pwr = reg_read(0x4E)
cfg = reg_read(0x50)
print(f"PWR_MGMT0     = {pwr:#04x}  (wrote 0x0F accel-LN+gyro-LN)")
print(f"ACCEL_CONFIG0 = {cfg:#04x}  (expect 0x06)")
print()

# ── Sample loop ────────────────────────────────────────────────────────────────
print("Sampling 20 x (waiting up to 10 ms each for DRDY)...")
print(f"{'#':>3}  {'DRDY':>5}  {'TEMP':>8}  {'AX':>8}  {'AY':>8}  {'AZ':>8}  {'GX':>8}  {'GY':>8}  {'GZ':>8}")
print("-" * 90)

for i in range(20):
    for _ in range(10):
        if reg_read(0x2D) & 0x08:
            break
        time.sleep_ms(1)

    status = reg_read(0x2D)
    t1, t0 = reg_read(0x1D), reg_read(0x1E)

    # Accel: 0x1F..0x24  (6 bytes)
    a = burst_read(0x1F, 6)
    ax = (a[0] << 8) | a[1]
    ay = (a[2] << 8) | a[3]
    az = (a[4] << 8) | a[5]

    # Gyro: 0x25..0x2A  (6 bytes)
    g = burst_read(0x25, 6)
    gx = (g[0] << 8) | g[1]
    gy = (g[2] << 8) | g[3]
    gz = (g[4] << 8) | g[5]

    drdy = 'DRDY' if (status & 0x08) else '    '
    print(f"{i:>3}  {drdy}  "
          f"{t1:02x}{t0:02x}  "
          f"{ax:#06x}  {ay:#06x}  {az:#06x}  "
          f"{gx:#06x}  {gy:#06x}  {gz:#06x}")
    time.sleep_ms(2)

print()
print("Key:")
print("  Accel 0x8000 + Gyro valid  → accel MEMS/ADC damaged")
print("  Accel 0x8000 + Gyro 0x8000 → shared analog supply issue (add decoupling cap)")
print("  Both valid                 → working!")
