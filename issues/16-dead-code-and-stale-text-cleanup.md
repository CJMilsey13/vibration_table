# Cleanup: dead code and a stale tooltip

**Severity:** Low
**Location:** `imu_visualizer.py`
**Status:** Confirmed by reading.

## Items

| Line | Item |
|---|---|
| 1664 | `_update_fs` is never called |
| 1326 | `_fs_history` is never used |
| 1307 | `_fft_done` signal is declared and never connected or emitted |
| 260, 295, 305 | `time.monotonic()` is called per frame / per batch for `t0`/`t1` values that `_on_batch` ignores |
| 1454 | Plot-length tooltip says "FFT always uses the full ring buffer"; Welch now covers the most recent `CTRL_SAMPLES` only |

The per-frame `monotonic()` call at line 260 runs 8000 times per second inside the serial parse loop.

## Acceptance

Items removed or corrected; the app starts in demo mode and with a serial connection as before.

## Resolution (2026-10-07)

**Fixed.**

`_update_fs`, `_fs_history`, `_fft_done` and the unused timestamps are removed; the tooltip is corrected.

Tests: `test_gui.py::test_dead_code_is_gone`
