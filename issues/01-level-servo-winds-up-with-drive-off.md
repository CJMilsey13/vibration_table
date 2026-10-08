# Level servo winds output to 0 dBFS while the drive is off

**Severity:** High (safety — rig can be hit at full scale)
**Location:** `imu_visualizer.py:2001-2004` (`_on_fft_done`), `imu_visualizer.py:1148-1155` (`nudge_level_db`)
**Status:** Reproduced by running the code; confirmed by reading.

## Problem

The level servo runs whenever the loop is enabled and control channels are selected. It does not check that an audio worker exists. With the drive off, the measured PSD is ambient noise, so `common` is large and positive and `nudge_level_db` adds `CTRL_LEVEL_MAX_STEP_DB` (+6 dB) every control update.

`_reset_correction` does not re-seed the level, so Reset and test start do not clear the wound-up value.

## Reproduction

Loop enabled, sensor connected, `_audio_worker is None`:

```
-20 -> -14 -> -8 -> -2 -> 0 dB   (4 control updates, about 8 s)
```

The slider still sits at -20; only the label shows the live value. The next Drive On or Start applies output gain 1.0. At 0 dB, 31.9 % of samples hard-clip.

## Suggested fix

- Skip the level servo (and the shape integrator) when `_audio_worker is None`.
- Re-seed `_level_db` from the slider on Drive On, on test start and on Reset.

## Acceptance

With the loop enabled and the drive off for 60 s, `level_db` equals the slider value.

## Resolution (2026-10-07)

**Fixed.**

`SpectralController.update(..., drive_active)` integrates nothing when the drive is off or paused; Drive On and Reset re-seed the level from the slider.

Tests: `test_control.py::test_drive_off_integrates_nothing`, `test_gui.py::test_loop_on_with_drive_off_leaves_the_level_alone`, `test_gui.py::test_drive_on_and_reset_start_from_the_slider`
