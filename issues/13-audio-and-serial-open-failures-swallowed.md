# Audio and serial open failures are only printed; the UI shows a working connection

**Severity:** Low-medium (operator believes the rig is driven or measured when it is not)
**Location:** `imu_visualizer.py:452-453` (`AudioOutputWorker.run`), `274-275` (`SerialWorker.run`), `1157-1177` (`_on_audio_start`), `1843-1846` (`_toggle_connection`)
**Status:** Confirmed by reading; reported as verified by the review run.

## Problem

Both workers catch the open exception and `print` it. The thread then exits.

- Audio: `_on_audio_start` has already stored the worker and switched the buttons to the Drive On state. The UI shows the drive running with a dead worker. This is also one way to reach the level-servo wind-up in ticket 01.
- Serial: the status label is set to the green port name before the worker has opened the port. A wrong or busy port shows as connected.

## Suggested fix

Add a `failed = pyqtSignal(str)` to each worker. On failure, emit it; the UI handler reverts the buttons and status and shows the message in the status bar.

## Acceptance

Selecting an invalid audio device or a busy COM port returns the UI to the off / disconnected state with a visible error.

## Resolution (2026-10-07)

**Fixed.**

Both workers emit `failed(str)`. Audio failure turns the drive off, stops a running test and shows the reason; serial failure returns the toolbar to Connect with "Connection failed".

Tests: `test_gui.py::test_audio_failure_turns_the_drive_off_and_says_so`, `::test_serial_worker_reports_a_port_it_cannot_open` (a real open of a port that does not exist), `::test_serial_failure_returns_the_ui_to_disconnected`
