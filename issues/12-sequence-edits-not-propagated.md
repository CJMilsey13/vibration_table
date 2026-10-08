# Sequence edits do not update the Grms plan; deleting a row mid-test hangs the test

**Severity:** Low-medium
**Location:** `imu_visualizer.py:686-687` (signal wiring), `1078-1081` (`_tick`)
**Status:** Confirmed by reading; reported as verified by the review run.

## Problem

1. The sequence model's `layoutChanged` and `dataChanged` are connected only to `_update_total_label`. They do not emit `profile_changed`, so the precomputed Grms demand staircase stays stale after a sequence edit. CLAUDE.md states "editing the table or sequence updates the plan immediately".

2. `_tick` returns early when `_step_idx >= len(steps)`. If a row is deleted while the test runs and the index is now past the end, the timer keeps firing, `_running` stays True and the test never completes.

## Suggested fix

- Connect the sequence model's change signals to a handler that emits `profile_changed`.
- In `_tick`, call `_complete_test()` when the index is out of range, or disable sequence editing while a test runs.

## Acceptance

Editing a step's gain or duration redraws the demand staircase at once. Deleting the current step mid-test ends or advances the test cleanly.

## Resolution (2026-10-07)

**Fixed.**

Sequence edits emit `sequence_changed`, which redraws the plan without resetting the correction. Deleting the step being run ends the test.

Tests: `test_gui.py::test_sequence_edit_redraws_the_plan_without_resetting_the_loop`, `::test_deleting_the_running_step_ends_the_test`; `test_sequence.py`
