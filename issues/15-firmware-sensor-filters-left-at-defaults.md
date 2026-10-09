# Firmware: accelerometer anti-alias and UI filters are left at reset defaults

**Severity:** Unverified (check the datasheet before acting)
**Location:** `firmware/main.c`, `icm_init` — no write to the accel filter registers
**Status:** Not verified. The register defaults below are from memory, not from the datasheet.

## Concern

The firmware never configures the accelerometer's anti-alias filter or UI filter bandwidth. If the reset defaults are roughly 1.16 kHz for the anti-alias filter and ODR/4 = 2 kHz for the UI filter, the sensor rolls off inside the 20-2000 Hz profile. The control loop would then read the top of the band low and over-drive it to compensate, and that over-drive would be baked into any saved speaker response.

## What to check

1. In the ICM-42688-P datasheet, read the reset values of `GYRO_ACCEL_CONFIG0` (bank 0) and `ACCEL_CONFIG_STATIC2/3/4` (bank 2), and the resulting bandwidths at 8 kHz ODR.
2. Compare with a reference accelerometer on the rig, or drive a swept sine and look for a roll-off above 1 kHz that the reference does not show.

## If confirmed

Set the filters explicitly in `icm_init` so the pass band covers the profile, and record the choice in CLAUDE.md. Bank-2 writes need `REG_BANK_SEL` changed and restored; do this before step 7 (`PWR_MGMT0`) and keep the existing order of the other steps.

## Resolution (2026-10-08)

**Confirmed and fixed as far as the sensor allows.** Firmware `2026-10-08`. The remainder is tracked in #17.

**Confirmed.** The firmware now reads the filter registers after reset and reports them. On this chip they are `GYRO_ACCEL_CONFIG0 = 0x11`, `ACCEL_CONFIG1 = 0x0D`, `ACCEL_CONFIG_STATIC2/3/4 = 0x30 / 0x40 / 0x62`: anti-alias filter DELT 24 (about 1.2 kHz, 2nd order) and UI filter at ODR/4. That is inside a 20–2000 Hz profile.

**Fixed.** Both are opened to their widest before the sensors are enabled: anti-alias filter DELT 63 / DELTSQR 3968 / BITSHIFT 3 (3979 Hz) and UI filter at ODR/2. The values are read back; a mismatch halts on blink code 4.

Sensor response, measured on the rig from the at-rest noise floor (`python tools/hw_check.py noise COM7`), dB relative to 100–300 Hz, mean of three axes:

| Band (Hz) | Old (reset defaults) | New |
|---|---|---|
| 300–600 | −0.4 | −0.1 |
| 600–1000 | −1.7 | −0.3 |
| 1000–1400 | −4.6 | −0.9 |
| 1400–2000 | −9.2 | −1.9 |
| 2000–3000 | −16.2 | −3.8 |
| 3000–3900 | −22.4 | −7.8 |

DC gain is unchanged: gravity reads 0.9997–1.0002 g with every setting tried.

**Not fixable by configuration.** Five other settings were flashed and measured (UI filter order 1/2/3, UI bandwidth codes 14 and 15, anti-alias filter off). All gave the same response below 2 kHz, to 0.1 dB. The last 1.9 dB at 1.4–2 kHz belongs to the sensor's fixed decimation at 8 kHz output rate. See #17 for the options.

**Trade-off.** The wider anti-alias filter rejects less above 4 kHz. This was not measured (it needs a source above 4 kHz).

Method check: the default filter's measured −4.6 dB at 1.0–1.4 kHz matches what a 2nd-order filter at DELT 24 plus the fixed stage predicts, so the white-noise-floor method reads a known filter correctly.
