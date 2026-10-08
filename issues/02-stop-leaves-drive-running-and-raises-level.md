# Stop / test completion leaves the drive running and the level rises

**Severity:** High (safety — shaker gets louder after Stop)
**Location:** `imu_visualizer.py:1053-1064` (`_stop_test`), `1107-1117` (`_complete_test`), `1263-1267` (`current_gain_db`), `1040-1051` (`_pause_test`)
**Status:** Reproduced by running the code; confirmed by reading.

## Problem

`_stop_test` stops only the step timer. The audio worker keeps running. `current_gain_db()` returns 0.0 as soon as `_running` is False, so the demand jumps to the 0 dB profile and the loop, still enabled, drives the rig up to meet it.

`_complete_test` does the same on a normal finish. Pause stops only the clock; drive and loop continue.

## Reproduction

- Tracking a -6 dB step, press Stop: the level servo raises the drive +6.8 dB over the next two updates and holds it.
- A sequence that ends at -12 dB jumps +12 dB on completion.

## Suggested fix

Decide what Stop means and make it explicit. Options:

1. Stop and completion also stop the drive (`_on_audio_stop`).
2. Keep the drive running but hold the last step's gain, and freeze the loop.

CLAUDE.md currently documents "audio reverts to 0 dB shape (test sequence ends, drive continues)". That text predates the level servo; with the servo, "reverts to 0 dB" is a real level increase. Update the doc with whichever behaviour is chosen.

## Acceptance

After Stop or completion, the measured Grms never exceeds the last step's level.

## Resolution (2026-10-07)

**Fixed.**

Stop and completion stop the drive (`_end_test`). Pause is a hold: drive keeps its level, loop frozen.

Tests: `test_gui.py::test_stop_stops_the_drive`, `::test_completion_stops_the_drive`, `::test_level_cannot_rise_after_stop`, `::test_pause_holds_level_and_shape`; `test_closed_loop.py` (drive off after the last step)
