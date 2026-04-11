# MicroPython firmware for ESP32-C3 SuperMini + MPU-9250 over SPI
# Binary output: 0xAA 0x55 + 3×float32 LE (ax, ay, az)  at 500 Hz
#
# Wiring:
#   MPU-9250 VCC  → 3.3 V
#   MPU-9250 GND  → GND
#   MPU-9250 SCLK → GPIO 4
#   MPU-9250 SDI  → GPIO 6   (MOSI)
#   MPU-9250 SDO  → GPIO 5   (MISO)
#   MPU-9250 NCS  → GPIO 7   (chip select, active low)
#   MPU-9250 FSYNC→ GND      (tie low)
#
# ICM-42688-P swap: wiring is identical — only register addresses change.

from machine import SPI, Pin
import struct
import sys
import time

# ── Configuration ─────────────────────────────────────────────────────────────
SCLK_PIN  = 4
MOSI_PIN  = 6
MISO_PIN  = 5
CS_PIN    = 7
SAMPLE_S  = 0.002   # 500 Hz

# MPU-9250 register addresses
REG_PWR_MGMT_1 = 0x6B
REG_ACCEL_CFG  = 0x1C
REG_GYRO_CFG   = 0x1B
REG_ACCEL_START = 0x3B  # ACCEL_XOUT_H — 6 bytes: ax, ay, az only

READ  = 0x80
WRITE = 0x00

ACCEL_LSB = 4096.0   # ±8 g → 4096 LSB/g

# Binary frame constants
SYNC = b'\xaa\x55'
_out = sys.stdout.buffer

# ── SPI helpers ───────────────────────────────────────────────────────────────
spi = SPI(1, baudrate=8_000_000, polarity=1, phase=1,
          sck=Pin(SCLK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
cs  = Pin(CS_PIN, Pin.OUT, value=1)

def write_reg(reg: int, val: int) -> None:
    cs(0)
    spi.write(bytes([WRITE | reg, val]))
    cs(1)

def read_regs(reg: int, n: int) -> bytes:
    cs(0)
    spi.write(bytes([READ | reg]))
    buf = spi.read(n)
    cs(1)
    return buf

# ── Initialise sensor ─────────────────────────────────────────────────────────
write_reg(REG_PWR_MGMT_1, 0x00)   # Wake up
time.sleep(0.1)
write_reg(REG_PWR_MGMT_1, 0x01)   # PLL clock
write_reg(REG_ACCEL_CFG,  0x10)   # ±8 g  (AFS_SEL = 2)
write_reg(REG_GYRO_CFG,   0x00)   # ±250 °/s (not used, kept in reset state)
time.sleep(0.1)

# ── Streaming loop ────────────────────────────────────────────────────────────
_pack = struct.pack
while True:
    raw = read_regs(REG_ACCEL_START, 6)          # 6 bytes: ax, ay, az only
    ax16, ay16, az16 = struct.unpack('>3h', raw)
    ax = ax16 / ACCEL_LSB
    ay = ay16 / ACCEL_LSB
    az = az16 / ACCEL_LSB
    _out.write(SYNC + _pack('<3f', ax, ay, az))  # 14 bytes total, no formatting cost
    time.sleep(SAMPLE_S)
