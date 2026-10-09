# Sensor response is still 1.9 dB down at 1.4–2 kHz; not configurable at 8 kHz output rate

**Severity:** Medium (the top of a 2 kHz profile is driven about 2 dB harder than indicated)
**Location:** `firmware/main.c` (sensor configuration), `control.py` / `stream.py` if corrected on the host
**Status:** Measured on the hardware. Found while fixing #15.

## Problem

With the accelerometer's filters opened as far as they go (#15), the sensor's response, measured from the at-rest noise floor, is:

| Band (Hz) | dB relative to 100–300 Hz |
|---|---|
| 600–1000 | −0.3 |
| 1000–1400 | −0.9 |
| 1400–2000 | −1.9 |
| 2000–3000 | −3.8 |

The control loop reads this as the rig's response and drives the top of the band harder to compensate. So the true level at 1.4–2 kHz is about 1.9 dB above what is displayed, and above demand.

Six filter settings were flashed and measured (UI filter order 1/2/3, UI bandwidth codes 0, 14 and 15, anti-alias filter on and off). Below 2 kHz they are identical to 0.1 dB. The droop belongs to the sensor's fixed decimation at 8 kHz output rate.

A second, related cost: with the anti-alias filter at its widest, content above 4 kHz is rejected less than before. Content at 6–8 kHz folds back into 0–2 kHz. This has not been measured.

## Options

1. **Oversample in firmware.** Run the sensor at 32 kHz output rate, where the same fixed stage sits four times higher, and decimate to 8 kHz in the MCU with a proper FIR. Fixes both the droop and the alias rejection at the source. Largest change: a new sample path at 32 kHz and a fixed-point filter.
2. **Correct on the host.** Divide the measured PSD by the sensor's measured response. Small change, but the correction is only as good as the noise-floor measurement, and it must be tied to the firmware's filter settings (the status line reports them).
3. **Accept and document.** Treat readings above 1 kHz as up to 2 dB low. This is the state today; CLAUDE.md says so.

## How to measure

`python tools/hw_check.py noise <port>` with the rig still and the drive off. Flat means every band within a few tenths of a dB of 100–300 Hz.
