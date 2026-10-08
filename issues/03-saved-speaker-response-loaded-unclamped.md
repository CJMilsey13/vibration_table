# Saved speaker response is loaded with no clamp; file on disk reaches +120 dB

**Severity:** High (safety / data — bad file is auto-applied at startup)
**Location:** `imu_visualizer.py:1786-1811` (`_load_response`), `speaker_response.json`
**Status:** Reproduced by running the code; file contents checked independently.

## Problem

`_load_response` re-grids the saved curve and applies it directly. Neither `max_correction_db` nor `CTRL_MAX_CUT_DB` is applied, and there is no sanity check on the values.

The file is auto-loaded at startup into `_H_base_db` and pushed to the audio worker on Drive On, even with the loop disabled. Reset and test start fall back to it. The loop's own clamp only takes effect after the first control update with the loop enabled.

The `speaker_response.json` currently on disk is a wound-up artifact from before the asymmetric clamp:

| | value |
|---|---|
| min | -22.3 dB |
| max | +120.0 dB |
| mean, 20-2000 Hz | 103.1 dB |

## Reproduction

Through `_generate_shaped_block` with the on-disk file:

| band | power with file | power, flat profile |
|---|---|---|
| 20-100 Hz | 0.87 % | 4.35 % |
| 100-500 Hz | 2.62 % | 20.11 % |
| 500-1000 Hz | 2.39 % | 25.00 % |
| 1000-1700 Hz | 46.97 % | 35.33 % |
| 1700-2000 Hz | 47.15 % | 15.22 % |

Per-bin drive spread is 139 dB.

## Suggested fix

- Clamp on load to `[-CTRL_MAX_CUT_DB, +max_correction_db]`, and remove the mean (a uniform offset is divided out by the unit-RMS normalisation anyway).
- Refuse, with a warning, a file whose spread exceeds a sane limit.
- Delete or regenerate the current `speaker_response.json`.
- Add `speaker_response.json` to `.gitignore`; it is rig-specific and untracked today, so `git add .` would commit it.

## Acceptance

Loading the current file either is refused or produces a correction inside the clamp.

## Resolution (2026-10-07)

**Fixed in code. The file itself was left in place.**

`load_response` refuses a curve with more than 5 % of bins outside the loop clamp and clips the rest; malformed files are refused. `speaker_response.json` is in `.gitignore`. The wound-up file on disk is now refused at startup (status bar says so) and the loop starts flat. It was not deleted or overwritten: relearn and press Save to replace it.

Tests: `test_control.py::test_wound_up_response_is_refused`, `::test_the_response_file_on_disk_is_refused`, `::test_sane_response_loads_clamped_and_tapered`, `::test_malformed_response_is_refused`; `test_gui.py::test_startup_refuses_a_wound_up_response`
