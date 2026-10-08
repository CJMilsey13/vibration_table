# After reconnect, spectrum and control loop stay dead for the length of the previous session

**Severity:** Medium (silent loss of measurement and control)
**Location:** `imu_visualizer.py:1829` (`_toggle_connection`), gates at `1885-1888` and `1927-1929`
**Status:** Reproduced by running the code; confirmed by reading.

## Problem

Disconnect resets `_n_samples` to 0 but leaves `_last_welch_n` and `_last_ctrl_n` at their old values. The gate

```python
self._n_samples - self._last_welch_n >= DISPLAY_WELCH_SAMPLES
```

is negative until the new session's sample count passes the old one. No Welch runs, so the PSD, in-spec status, Grms readout and control loop are all frozen. The status shows connected, the drive may be on, and `_psd` keeps showing the stale spectrum.

## Reproduction

After a 60 s session and reconnect, the first Welch launches at t = 60.03 s instead of 2.0 s. A 10-minute run gives a 10-minute blackout.

## Suggested fix

On disconnect also reset `_last_welch_n`, `_last_ctrl_n`, `_ring_ptr`, `_psd` and `_fft_running`. Consider one `_reset_stream_state()` used by connect and disconnect.

## Acceptance

After any session length, the first Welch after reconnect fires at 2.0 s.

## Resolution (2026-10-07)

**Fixed.**

`MeasurementStream.reset()` clears data, filter state and both Welch gates, and is called on connect. A PSD computed from the previous session is discarded.

Tests: `test_stream.py::test_reset_restores_both_gates`, `test_gui.py::test_reconnect_does_not_blank_the_spectrum`, `::test_result_from_a_previous_session_is_discarded`
