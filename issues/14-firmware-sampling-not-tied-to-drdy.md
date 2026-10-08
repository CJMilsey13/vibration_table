# Firmware: sampling is timed by the MCU clock, not by the sensor's data-ready

**Severity:** Needs measurement (code path confirmed; size of the effect not measured)
**Location:** `firmware/main.c:358-365` (polling loop), `4`, `36`, `85`, `91` (unused INT1 comment and defines)
**Status:** Confirmed by reading only. Not run on hardware.

## Problem

Core 0 reads the accel registers every 125 µs on the MCU timer. The sensor produces samples at 8 kHz on its own oscillator. The two clocks are not locked, so the read drifts through the sensor's sample period. When the sensor is slower a sample is read twice; when it is faster a sample is skipped. The rate of these events is the frequency difference between the two clocks.

`PIN_INT1`, `ICM_INT1_PP_ACTIVE_HIGH` and `ICM_DRDY_INT1_EN` are defined but never used, and the file header still says Core 0 "handles INT1 GPIO interrupt at 8 kHz".

## What to check first

Log raw frames with the rig still and count exact consecutive repeats across all three axes. A steady rate of repeats confirms the effect and gives the clock offset.

## Suggested fix

Route DRDY to INT1 and sample on the interrupt (or poll the INT pin), so there is one read per sensor sample. The init sequence in CLAUDE.md is marked "do not reorder"; add the INT configuration without changing the order of the existing steps, and update the doc.

Alternatively, correct the header comment and remove the unused defines if polling is the intended design.

## Resolution (2026-10-07)

**Open — needs a capture from the rig.**

The firmware header comment now says sampling is polled. `tools/count_repeats.py <port>` was added to measure the repeat rate with the rig still. No change to the sampling itself.

Tests: `test_serial.py::test_count_repeats_finds_duplicated_samples` (the tool, on synthetic frames)
