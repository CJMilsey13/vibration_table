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

## Resolution (2026-10-07)

**Open — not verified.**

The datasheet could not be retrieved in this session, so the reset values are still unconfirmed. No change made.

Tests: None
