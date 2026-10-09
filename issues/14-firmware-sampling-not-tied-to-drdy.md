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

## Resolution (2026-10-08)

**Fixed and verified on the hardware.** Firmware `2026-10-08` plus a host change.

**What was measured.** The sensor does not run at 8000 Hz. Its 8 kHz output rate comes from its internal oscillator and is **8107.41 Hz** on this unit (firmware, against the MCU crystal; 8107.8 Hz by the PC clock, so two clocks agree to 50 ppm). The old firmware read at 8000.1 Hz, so about **107 samples every second were never read**, and about 5 a second were read twice. This was worse than the ticket supposed.

**Firmware.** Core 0 now reads exactly one sample per sensor data-ready, by polling `UI_DRDY` in `INT_STATUS` over SPI. No INT1 wire is needed. It measures the sensor's rate and reports it every 2 s in a text status line between frames (`odr=8107.411`).

**Host.** The stream now arrives at the sensor's rate, so `MeasurementStream` resamples it to exactly 8000 Hz using the reported rate. Without that, every frequency would read 1.33 % low.

Measured on the rig:

| | Old firmware | New firmware |
|---|---|---|
| Frames per second (PC clock) | 8000.14 | 8107.75 |
| Samples read twice (excess over chance) | 5.3 per second | 0.00 per second |
| Samples missed | ~107 per second, unreported | 0 in 1,459,481 frames (180 s) |
| Sidebands ±107 Hz beside the 1737 Hz ambient line | +3.0 / +2.3 dB over the floor | +0.1 / +0.3 dB |
| Stream rate after the host resampler (PC clock) | n/a | 8000.26 per second (+33 ppm) |

Frequency axis, checked with ambient lines on the rig: 1737 and 2606 Hz under the old firmware; 1714 and 2571 Hz from the new firmware unresampled (ratio 0.9867, as predicted by 8000/8107.4); 1737 and 2606 Hz again through the real `SerialWorker` and resampler.

Resampler: a tone keeps its frequency to 0.02 Hz and its level to 0.01 dB from 20 Hz to 3 kHz; flat to 3.3 kHz; nothing above 4 kHz folds back (−89 dB).

Tests: `tests/test_resample.py` (18 tests), `tests/test_serial.py::test_status_line_*`, `tests/test_gui.py::test_status_line_sets_the_stream_rate_and_says_so`, `::test_a_new_connection_starts_from_the_nominal_rate_again`, `::test_sensor_rate_stream_shows_a_tone_at_its_true_frequency`.

Do not use the new firmware with a visualizer older than this change: it would show every frequency 1.33 % low.
