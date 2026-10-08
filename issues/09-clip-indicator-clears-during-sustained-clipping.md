# CLIP indicator disappears after 2 s while the output is still clipping

**Severity:** Medium (hides distortion the operator needs to see)
**Location:** `imu_visualizer.py:425-431` (audio callback), `1188-1190` (`_on_clip_detected`)
**Status:** Reproduced by running the code; confirmed by reading.

## Problem

`clipped_last` suppresses `clip_detected` while consecutive blocks clip, so the signal fires only on the transition into clipping. The label auto-clears on a 2 s timer. During sustained clipping the label is blank.

The level servo may raise the level to 0 dB, so this state is reachable in normal closed-loop use.

## Reproduction

200 consecutive clipping blocks (about 18.6 s) at 0 dB emit the signal once.

| level | samples clipped |
|---|---|
| 0 dB | 31.9 % |
| -6 dB | 4.6 % |
| -10 dB | 0.15 % |
| -12 dB | 0.007 % |

## Suggested fix

Emit on every clipping block (or rate-limit to a few per second) and let each emission restart the clear timer, so the label stays on while clipping continues. Consider capping the level servo below the point where a unit-RMS Gaussian block clips (about -12 dB).

## Acceptance

The CLIP label stays visible for the whole of a sustained clipping period and clears 2 s after it ends.

## Resolution (2026-10-07)

**Fixed.**

`clip_detected` fires for every clipping block and each one restarts the 2 s clear timer.

Tests: `test_drive.py::test_every_clipping_block_is_flagged`, `test_gui.py::test_clip_label_stays_lit_while_clipping_continues`
