"""MeasurementStream: the two Welch gates and the reset that ticket 05 lacked."""

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

from stream import (
    CTRL_SAMPLES, CTRL_SETTLE_SAMPLES, DISPLAY_WELCH_SAMPLES, HISTORY, PSD_FMAX,
    PSD_FMIN, SAMPLE_RATE, MeasurementStream,
)

BATCH = 80


def run(stream, seconds, rng=None):
    """Push `seconds` of data; return [(n_samples at take, window), ...]."""
    taken = []
    for _ in range(int(seconds * SAMPLE_RATE / BATCH)):
        batch = (rng.normal(size=(BATCH, 3)) if rng is not None
                 else np.zeros((BATCH, 3))).astype(np.float32)
        stream.push(batch)
        w = stream.take_window()
        if w is not None:
            taken.append((stream.n_samples, w))
    return taken


def test_first_window_arrives_at_two_seconds_and_is_a_control_window():
    taken = run(MeasurementStream(3), 3.0)
    n, w = taken[0]
    assert n == CTRL_SAMPLES
    assert w.is_control
    assert w.samples.shape == (CTRL_SAMPLES, 3) and w.samples.dtype == np.float64


def test_control_windows_never_share_samples():
    taken = run(MeasurementStream(3), 30.0)
    ends = [n for n, w in taken if w.is_control]
    assert len(ends) >= 13                           # one per 2 s, give or take the gate
    # A window covers (end - CTRL_SAMPLES, end], so no overlap ⇔ ends ≥ CTRL_SAMPLES apart.
    assert min(np.diff(ends)) >= CTRL_SAMPLES


def test_display_windows_come_much_faster_than_control_windows():
    taken = run(MeasurementStream(3), 12.0)
    display = [n for n, w in taken]
    control = [n for n, w in taken if w.is_control]
    assert min(np.diff(display)) >= DISPLAY_WELCH_SAMPLES
    assert len(display) > 10 * len(control)          # ~14 Hz against 0.5 Hz


def test_reset_restores_both_gates():
    """Ticket 05: after a 60 s session the next one must not wait 60 s."""
    s = MeasurementStream(3)
    run(s, 60.0)
    epoch = s.epoch
    s.reset()
    assert s.n_samples == 0 and s.epoch == epoch + 1
    taken = run(s, 5.0)
    n, w = taken[0]
    assert n == CTRL_SAMPLES                         # 2.0 s, not 60 s
    assert w.is_control and w.epoch == epoch + 1


def test_restart_makes_the_next_control_window_start_after_the_change():
    s = MeasurementStream(3)
    run(s, 9.0)                                      # mid-stream, control window almost due
    at = s.n_samples
    s.restart_control_window()
    taken = run(s, 6.0)
    first_ctrl = next(n for n, w in taken if w.is_control)
    # The window is (first_ctrl - CTRL_SAMPLES, first_ctrl]: all of it after `at`,
    # with the settle guard on top.
    assert first_ctrl - CTRL_SAMPLES >= at + CTRL_SETTLE_SAMPLES
    assert first_ctrl - CTRL_SAMPLES < at + CTRL_SETTLE_SAMPLES + 2 * DISPLAY_WELCH_SAMPLES
    # The display never paused for it.
    assert len([n for n, w in taken if n <= first_ctrl]) > 25


def test_window_taken_before_a_restart_is_no_longer_a_control_window():
    s = MeasurementStream(3)
    w = next(w for _, w in run(s, 3.0) if w.is_control)
    assert s.is_control_window(w) and s.is_current(w)
    s.restart_control_window()
    assert not s.is_control_window(w)
    assert s.is_current(w)                           # still fine for the display
    s.reset()
    assert not s.is_current(w) and not s.is_control_window(w)


def test_window_is_the_bandpassed_tail_across_the_ring_wrap(rng):
    s = MeasurementStream(3)
    seconds = 11.0                                   # > HISTORY, so the ring has wrapped
    data = rng.normal(size=(int(seconds * SAMPLE_RATE), 3)).astype(np.float32)
    last = None
    for i in range(0, len(data), BATCH):
        s.push(data[i:i + BATCH])
        w = s.take_window()
        if w is not None:
            last = (s.n_samples, w)
    assert s.n_samples > HISTORY
    n, w = last
    sos  = butter(4, [PSD_FMIN, min(PSD_FMAX - 5.0, SAMPLE_RATE * 0.49)],
                  btype='bandpass', fs=SAMPLE_RATE, output='sos')
    # Same initial filter state the stream uses (steady state for a 1 g input).
    zi   = np.stack([sosfilt_zi(sos)] * 3, axis=-1)
    ref, _ = sosfilt(sos, data[:n].astype(np.float64), axis=0, zi=zi)
    assert np.allclose(w.samples, ref[n - CTRL_SAMPLES:n], atol=1e-4)


def test_recent_raw_is_the_unfiltered_tail_in_order(rng):
    s = MeasurementStream(3)
    data = rng.normal(size=(HISTORY + 4000, 3)).astype(np.float32)
    for i in range(0, len(data), BATCH):
        s.push(data[i:i + BATCH])
    assert np.array_equal(s.recent_raw(16000), data[-16000:])
    assert np.array_equal(s.recent_raw(HISTORY), data[-HISTORY:])
