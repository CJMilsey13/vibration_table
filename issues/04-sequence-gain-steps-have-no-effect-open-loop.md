# Test-sequence gain steps do nothing to the drive level in open loop

**Severity:** Medium (wrong behaviour; UI shows a level change that does not happen)
**Location:** `imu_visualizer.py:507-509` (`_generate_shaped_block`), `1257-1261` (`_push_audio_profile`)
**Status:** Reproduced by running the code; confirmed by reading.

## Problem

`_push_audio_profile` sends each step's `gain_db` to the worker, where it scales `psd_vals`. The block is then normalised to unit RMS, which divides the gain straight back out. Only `output_gain` changes the level.

With the loop enabled the level servo hides this, because it chases the new demand. With the loop disabled, every step drives the shaker identically while the step label, target curve and Grms staircase show a change.

## Reproduction

```
gain_db=-12.0  block RMS mean=1.000000
gain_db= -6.0  block RMS mean=1.000000
gain_db= +0.0  block RMS mean=1.000000
gain_db= +6.0  block RMS mean=1.000000
```

## Suggested fix

Apply the step gain as an amplitude factor after normalisation (`10 ** (gain_db / 20)`), or fold it into the output level on each step change. Check the interaction with the level servo so the step is not applied twice in closed loop.

## Documentation

The CLAUDE.md failure-mode row "Audio level not changing between steps" blames Drive On being clicked after Start. That is not the cause. Correct the row when this is fixed.

## Acceptance

Open loop, a -6 dB step followed by a 0 dB step changes block RMS by 6.0 dB.

## Resolution (2026-10-07)

**Fixed.**

A step's gain is applied as output level: `output_dbfs(gain_db)` through `_apply_level()`. In closed loop this is a feedforward; the servo trims only the remainder.

Tests: `test_gui.py::test_open_loop_sequence_steps_change_the_drive_level` (6.00 dB), `test_control.py::test_gain_step_moves_output_by_the_step_and_needs_no_relearning`, `test_closed_loop.py`
