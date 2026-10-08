# Duplicate breakpoint frequency raises ZeroDivisionError in psd_grms

**Severity:** Low-medium (profile update aborts halfway; UI and loop disagree)
**Location:** `imu_visualizer.py:175` (`psd_grms`), reached from `BreakpointModel.setData` via `_on_profile_changed` (`988-991`)
**Status:** Reproduced by running the code; confirmed by reading.

## Problem

`psd_grms` divides by `math.log10(f2 / f1)`. Two breakpoints at the same frequency make that zero. The exception is raised inside `_on_profile_changed` before `_push_audio_profile` and `profile_changed.emit()`, so the audio profile and target curve keep the old table while the control loop already reads the new one.

A zero or negative level or frequency fails the same way in `log10`.

## Reproduction

```
psd_grms([(20, 1e-3), (20, 1e-3), (2000, 5e-4)])
-> ZeroDivisionError: float division by zero
```

## Suggested fix

Validate in `BreakpointModel.setData`: reject non-positive values and frequencies that are not strictly increasing. Make `psd_grms` skip zero-width segments as a second guard.

## Acceptance

Entering a duplicate frequency is rejected in the table and nothing raises.

## Resolution (2026-10-07)

**Fixed.**

The table rejects an edit that leaves duplicate or non-positive values; `psd_grms` skips zero-width segments.

Tests: `test_gui.py::test_duplicate_breakpoint_frequency_is_rejected`, `test_control.py::test_duplicate_breakpoint_frequency_does_not_raise`, `::test_breakpoint_error`
