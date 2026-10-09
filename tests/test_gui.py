"""The real MainWindow, offscreen, with no sound device and no serial port.

These check the wiring between the widgets and the Qt-free modules — the part
the module tests cannot see. Ticket numbers refer to issues/NN-*.md.
"""

import json

import numpy as np
import pytest
from PyQt5 import QtTest

from control import psd_grms, psd_interp_loglog
from sim import FREQS, MASK, PROFILE, noise, on_target, welch_full
from stream import CTRL_SAMPLES, SAMPLE_RATE

AMBIENT = np.full(len(FREQS), 4.2e-9)     # accelerometer noise floor, g²/Hz
BAND    = (FREQS >= 20.0) & (FREQS <= 2000.0)


def window(rng, offset_db=0.0, gain_db=0.0, chans=(2,)):
    """One Welch result (3 channels, full rfft grid). Driven channels sit
    offset_db from demand; offset_db=None means the drive is off."""
    out = []
    for ch in range(3):
        if offset_db is None or ch not in chans:
            true = AMBIENT
        else:
            true = np.where(BAND, psd_interp_loglog(FREQS, PROFILE, gain_db + offset_db), 0.0)
        out.append(welch_full(noise(true, rng)))
    return out


def set_sequence(dock, steps):
    m = dock._seq_model
    while m.rowCount() > len(steps):
        m.remove_row(m.rowCount() - 1)
    while m.rowCount() < len(steps):
        m.add_row()
    for r, (dur, gain) in enumerate(steps):
        assert m.setData(m.index(r, 1), str(dur))
        assert m.setData(m.index(r, 2), str(gain))


def out_dbfs(dock):
    return 20 * np.log10(dock._audio_worker._output_gain)


# ── Ticket 01 ────────────────────────────────────────────────────────────────

def test_loop_on_with_drive_off_leaves_the_level_alone(win, rng):
    d = win._profile_dock
    d._ctrl_enable_cb.setChecked(True)
    for _ in range(6):
        win._on_fft_done(window(rng, offset_db=None))
    assert win._ctl.level_db == d._audio_slider.value() == -20
    assert not np.any(win._ctl.corr_db)
    assert 'drive off' in d._ctrl_err_lbl.text()
    assert d._audio_level_lbl.text() == '-20.0 dB'


def test_drive_on_and_reset_start_from_the_slider(win, rng):
    d = win._profile_dock
    win._ctl.reseed_level(-3.0)                      # as if left there by an earlier run
    d._on_audio_start()
    assert out_dbfs(d) == pytest.approx(-20.0)

    d._ctrl_enable_cb.setChecked(True)
    for _ in range(3):
        win._on_fft_done(window(rng, offset_db=-9.0))
    assert out_dbfs(d) > -15.0                       # the servo did raise it...
    d._on_ctrl_reset()
    assert out_dbfs(d) == pytest.approx(-20.0)       # ...and Reset puts it back
    assert not np.any(win._ctl.corr_db)


# ── Ticket 02 ────────────────────────────────────────────────────────────────

def test_stop_stops_the_drive(win):
    d = win._profile_dock
    d._start_test()
    d._timer.stop()
    assert d.drive_running and d.is_running
    assert out_dbfs(d) == pytest.approx(-26.0)       # slider -20, step -6
    d._stop_test()
    assert not d.drive_running and not d.is_running
    assert d.current_gain_db() == 0.0
    assert d._start_btn.isEnabled() and not d._stop_btn.isEnabled()
    assert d._step_lbl.text() == 'Stopped'


def test_completion_stops_the_drive(win):
    d = win._profile_dock
    set_sequence(d, [(2, -12), (1, 0)])
    d._start_test()
    d._timer.stop()
    assert out_dbfs(d) == pytest.approx(-32.0)
    d._tick(); d._tick()                             # → step 2
    assert d.drive_running and out_dbfs(d) == pytest.approx(-20.0)
    d._tick()                                        # → complete
    assert not d.drive_running and not d.is_running
    assert d._step_lbl.text().startswith('Complete')


def test_level_cannot_rise_after_stop(win, rng):
    d = win._profile_dock
    d._ctrl_enable_cb.setChecked(True)
    d._start_test()
    d._timer.stop()
    for _ in range(4):
        win._on_fft_done(window(rng, gain_db=-6.0))  # tracking the -6 dB step
    level = win._ctl.level_db
    d._stop_test()
    for _ in range(6):
        win._on_fft_done(window(rng, offset_db=None))
    assert win._ctl.level_db == level


def test_pause_holds_level_and_shape(win, rng):
    d = win._profile_dock
    d._ctrl_enable_cb.setChecked(True)
    d._start_test()
    d._timer.stop()
    win._on_fft_done(window(rng, offset_db=-4.0, gain_db=-6.0))
    d._pause_test()
    held = (win._ctl.level_db, win._ctl.corr_db, out_dbfs(d))
    assert d.drive_running and d.is_paused
    for _ in range(4):
        win._on_fft_done(window(rng, offset_db=-4.0, gain_db=-6.0))
    assert win._ctl.level_db == held[0] and out_dbfs(d) == held[2]
    assert np.array_equal(win._ctl.corr_db, held[1])
    assert 'paused' in d._ctrl_err_lbl.text()
    d._pause_test()                                  # resume
    d._timer.stop()
    win._on_fft_done(window(rng, offset_db=-4.0, gain_db=-6.0))
    assert win._ctl.level_db > held[0]


# ── Ticket 03 ────────────────────────────────────────────────────────────────

def write_response(path, corr):
    path.write_text(json.dumps({'freqs_hz': list(map(float, FREQS)),
                                'corr_db': list(map(float, corr))}))


def test_startup_refuses_a_wound_up_response(iv):
    write_response(iv.RESPONSE_FILE, np.where(BAND, 103.0, 0.0))
    w = iv.MainWindow(demo=False)
    try:
        assert not np.any(w._ctl.corr_db) and not np.any(w._ctl.base_db)
        assert 'NOT loaded' in w.statusBar().currentMessage()
        w._profile_dock._on_audio_start()
        assert w._profile_dock._audio_worker._corr_db is None
    finally:
        w.close()


def test_startup_applies_a_sane_response_even_with_the_loop_off(iv):
    corr = np.where(BAND, 6.0 * np.sin(FREQS / 300.0), 0.0)
    write_response(iv.RESPONSE_FILE, corr)
    w = iv.MainWindow(demo=False)
    try:
        assert np.allclose(w._ctl.base_db, corr, atol=1e-6)
        assert 'loaded' in w.statusBar().currentMessage()
        w._profile_dock._on_audio_start()
        assert np.allclose(w._profile_dock._audio_worker._corr_db, corr, atol=1e-6)
    finally:
        w.close()


# ── Ticket 04 ────────────────────────────────────────────────────────────────

def test_open_loop_sequence_steps_change_the_drive_level(win, rng):
    d = win._profile_dock
    assert not d.loop_enabled
    set_sequence(d, [(1, -6), (1, 0)])
    d._start_test()
    d._timer.stop()

    def block_rms_db():
        x = np.concatenate([d._audio_worker.render(4096, rng)[0] for _ in range(20)])
        return 20 * np.log10(np.sqrt(np.mean(x.astype(np.float64) ** 2)))

    first = block_rms_db()
    assert d._audio_level_lbl.text() == '-26.0 dB'
    d._tick()
    second = block_rms_db()
    assert d._audio_level_lbl.text() == '-20.0 dB'
    assert second - first == pytest.approx(6.0, abs=0.01)


# ── Ticket 05 ────────────────────────────────────────────────────────────────

class RecordingPool:
    """Stands in for the thread pool: notes when a Welch would have launched."""

    def __init__(self, win):
        self.win, self.launched = win, []

    def start(self, _runnable):
        self.launched.append(self.win._stream.n_samples)
        self.win._fft_running = False

    def waitForDone(self, *_):
        return True


def feed_silence(win, seconds):
    batch = np.zeros((80, 3), dtype=np.float32)
    for _ in range(int(seconds * SAMPLE_RATE / 80)):
        win._on_batch(batch, 0)


def test_reconnect_does_not_blank_the_spectrum(win, iv, monkeypatch):
    monkeypatch.setattr(iv.DemoWorker, 'start', lambda self: None)
    pool = win._pool = RecordingPool(win)
    win._start_worker(iv.DemoWorker())
    feed_silence(win, 60.0)
    assert pool.launched[0] == CTRL_SAMPLES and len(pool.launched) > 500

    pool.launched.clear()
    win._start_worker(iv.DemoWorker())               # reconnect
    feed_silence(win, 5.0)
    assert pool.launched[0] == CTRL_SAMPLES          # 2.0 s — was 60 s


def take_control_window(win):
    """Feed silence until the stream hands out a control window."""
    batch = np.zeros((80, 3), dtype=np.float32)
    for _ in range(1000):
        win._stream.push(batch)
        w = win._stream.take_window()
        if w is not None and w.is_control:
            return w
    raise AssertionError('no control window in 10 s')


def test_result_from_a_previous_session_is_discarded(win, rng):
    d = win._profile_dock
    stale = take_control_window(win)
    win._stream.reset()
    win._on_welch_done((window(rng), stale))
    assert d._spec_status_lbl.text() == '—' and win._psd is None
    win._on_welch_done((window(rng), take_control_window(win)))
    assert d._spec_status_lbl.text() != '—' and win._psd is not None


# ── Data measured before a drive change must not be controlled on ────────────

def test_drive_on_mid_stream_does_not_act_on_pre_drive_data(win, rng):
    """Loop already enabled, sensor streaming, then Drive On: the window in
    hand is ambient noise and would read as a 40 dB shortfall."""
    d = win._profile_dock
    d._ctrl_enable_cb.setChecked(True)
    before = take_control_window(win)                # gathered with the drive off
    d._on_audio_start()
    win._on_welch_done((window(rng, offset_db=None), before))
    assert win._ctl.level_db == -20.0 and not np.any(win._ctl.corr_db)
    assert win._psd is not None                      # the display still got it

    after = take_control_window(win)                 # gathered entirely since Drive On
    assert win._stream.is_control_window(after)
    win._on_welch_done((window(rng, offset_db=-3.0), after))
    assert win._ctl.level_db == pytest.approx(-17.0, abs=0.3)


@pytest.mark.parametrize('change', ['step', 'slider', 'reset', 'sequence edit'])
def test_every_external_drive_change_restarts_the_control_window(win, change):
    d = win._profile_dock
    d._start_test()
    d._timer.stop()
    stale = take_control_window(win)
    assert win._stream.is_control_window(stale)
    {'step':          lambda: [d._tick() for _ in range(300)],
     'slider':        lambda: d._audio_slider.setValue(-25),
     'reset':         d._on_ctrl_reset,
     'sequence edit': lambda: d._seq_model.setData(d._seq_model.index(0, 2), '-9'),
     }[change]()
    assert d.drive_running
    assert not win._stream.is_control_window(stale)


def test_the_servo_moving_the_level_does_not_restart_the_window(win, rng):
    d = win._profile_dock
    d._ctrl_enable_cb.setChecked(True)
    d._on_audio_start()
    w = take_control_window(win)
    win._on_fft_done(window(rng, offset_db=-3.0))    # servo acts
    assert win._ctl.level_db > -20.0
    assert win._stream.is_control_window(w)          # loop cadence is untouched


# ── Ticket 06 and the Settings view ──────────────────────────────────────────

def test_settings_tab_holds_the_in_spec_averaging(win, rng):
    d = win._profile_dock
    assert [win._tabs.tabText(i) for i in range(win._tabs.count())] == ['Live', 'Settings']
    assert win._spec_avg_spin.value() == win._assessor.n_avg == 8
    assert '16 s' in win._spec_avg_lbl.text()

    win._spec_avg_spin.setValue(3)
    assert win._assessor.n_avg == 3 and '6 s' in win._spec_avg_lbl.text()
    win._on_fft_done(window(rng))
    assert d._spec_status_lbl.text().startswith('AVERAGING 1/3')
    win._on_fft_done(window(rng))
    win._on_fft_done(window(rng))
    assert not d._spec_status_lbl.text().startswith('AVERAGING')


def test_on_target_rig_reads_in_spec_with_the_loop_off(win, rng):
    d = win._profile_dock
    for _ in range(8):
        win._on_fft_done(window(rng))
    assert d._spec_status_lbl.text().startswith('IN SPEC')
    for _ in range(8):
        win._on_fft_done(window(rng, offset_db=+8.0))
    assert d._spec_status_lbl.text().startswith('ABORT')


def test_display_rate_windows_do_not_feed_status_or_loop(win, rng):
    d = win._profile_dock
    d._ctrl_enable_cb.setChecked(True)
    d._on_audio_start()
    for _ in range(10):
        win._on_fft_done(window(rng, offset_db=-9.0), is_control=False)
    assert d._spec_status_lbl.text() == '—'
    assert win._ctl.level_db == -20.0 and not np.any(win._ctl.corr_db)
    assert d._grms_meas_lbl.text() != 'Meas: — g'    # the readout does stay live


# ── Ticket 09 ────────────────────────────────────────────────────────────────

def test_clip_label_stays_lit_while_clipping_continues(win):
    d = win._profile_dock
    d._on_audio_start()
    w = d._audio_worker
    for _ in range(3):                               # 3.6 s of repeated clipping
        w.clip_detected.emit()
        QtTest.QTest.qWait(1200)
        assert d._audio_clip_lbl.text() == 'CLIP'    # used to clear at 2.0 s
    QtTest.QTest.qWait(1200)                         # 2.4 s after the last one
    assert d._audio_clip_lbl.text() == ''


# ── Ticket 10 ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('names', [('Accel Z',), ('Accel X', 'Accel Z'),
                                   ('Accel X', 'Accel Y', 'Accel Z')])
def test_measured_grms_matches_demand_for_any_number_of_channels(win, rng, names):
    d = win._profile_dock
    for name, cb in d._ch_checks.items():
        cb.setChecked(name in names)
    chans = tuple(i for i, n in enumerate(d._ch_checks) if n in names)
    win._on_fft_done(window(rng, chans=chans))
    meas = float(d._grms_meas_lbl.text().split()[1])
    assert meas == pytest.approx(psd_grms(PROFILE), rel=0.03)   # was √N too high


# ── Ticket 11 ────────────────────────────────────────────────────────────────

def test_duplicate_breakpoint_frequency_is_rejected(win):
    d = win._profile_dock
    m = d._bp_model
    before = m.breakpoints()
    assert m.setData(m.index(1, 0), '20') is False   # same as row 0
    assert m.setData(m.index(0, 1), '0') is False
    assert m.setData(m.index(0, 1), 'abc') is False
    assert m.breakpoints() == before
    assert m.setData(m.index(1, 0), '1000') is True  # a real edit still goes through
    assert d._grms_base_lbl.text() == f'Base Grms: {psd_grms(m.breakpoints()):.3f} g'


# ── Ticket 12 ────────────────────────────────────────────────────────────────

def test_sequence_edit_redraws_the_plan_without_resetting_the_loop(win, rng):
    d = win._profile_dock
    d._ctrl_enable_cb.setChecked(True)
    d._on_audio_start()
    tilt = np.where(FREQS > 500, -6.0, 0.0)
    for _ in range(4):
        true = np.where(BAND, psd_interp_loglog(FREQS, PROFILE) * 10 ** (tilt / 10), 0.0)
        z = welch_full(noise(true, rng))
        win._on_fft_done([z, z, z])
    learned = win._ctl.corr_db
    assert np.any(learned)

    set_sequence(d, [(30, -12), (300, 0)])
    t, g = win._grms_demand_curve.getData()
    assert list(t) == [0.0, 30.0, 30.0, 330.0]
    assert g[0] == pytest.approx(psd_grms(PROFILE, -12.0))
    assert np.array_equal(win._ctl.corr_db, learned)
    assert d._total_lbl.text() == 'Total: 5m 30s'


def test_deleting_the_running_step_ends_the_test(win):
    d = win._profile_dock
    set_sequence(d, [(1, -6), (60, 0)])
    d._start_test()
    d._timer.stop()
    d._tick()                                        # now on step 2
    assert d._runner.step_index == 1
    d._seq_model.remove_row(1)
    assert not d.is_running and not d.drive_running
    assert d._start_btn.isEnabled()


def test_editing_the_running_steps_gain_moves_the_drive(win):
    d = win._profile_dock
    d._start_test()
    d._timer.stop()
    assert out_dbfs(d) == pytest.approx(-26.0)
    d._seq_model.setData(d._seq_model.index(0, 2), '-10')
    assert out_dbfs(d) == pytest.approx(-30.0)


# ── Ticket 13 ────────────────────────────────────────────────────────────────

def test_audio_failure_turns_the_drive_off_and_says_so(win):
    d = win._profile_dock
    d._start_test()
    d._timer.stop()
    d._audio_worker.failed.emit('Invalid device')
    assert not d.drive_running and not d.is_running
    assert d._audio_start_btn.isEnabled() and not d._audio_stop_btn.isEnabled()
    assert 'Invalid device' in win.statusBar().currentMessage()


def test_serial_worker_reports_a_port_it_cannot_open(iv):
    w = iv.SerialWorker('COM_DOES_NOT_EXIST', 115200, 2048.0)
    got = []
    w.failed.connect(got.append)
    w.run()                                          # synchronously, in this thread
    assert len(got) == 1 and got[0]


def test_serial_failure_returns_the_ui_to_disconnected(win, iv, monkeypatch):
    monkeypatch.setattr(iv.SerialWorker, 'start', lambda self: None)
    sw = iv.SerialWorker('COM_DOES_NOT_EXIST', 115200, 2048.0)
    win._start_worker(sw)
    win._connect_btn.setText('Disconnect')
    sw.failed.emit('could not open port')
    assert win.worker is None
    assert win._connect_btn.text() == 'Connect'
    assert 'failed' in win._status_lbl.text()
    assert 'could not open port' in win.statusBar().currentMessage()


# ── Ticket 14: the sensor's own sample rate ──────────────────────────────────

BANNER = ('# icm42688_streamer 2026-10-08 drdy-polled odr=8107.419 '
          'reset=11,0D,30,40,62 now=01,0D,7E,80,3F')


def test_status_line_sets_the_stream_rate_and_says_so(win, iv, monkeypatch, rng):
    monkeypatch.setattr(iv.SerialWorker, 'start', lambda self: None)
    sw = iv.SerialWorker('FAKE', 115200, 2048.0)
    win._start_worker(sw)
    assert win._stream.input_rate == 8000.0 and 'configured' in win._fs_lbl.text()

    for _ in range(8):                               # something averaged before the line arrives
        win._on_fft_done(window(rng))
    assert win._profile_dock._spec_status_lbl.text().startswith('IN SPEC')
    epoch = win._stream.epoch

    sw.info_ready.emit(BANNER)
    assert win._stream.input_rate == 8107.419
    assert win._stream.epoch == epoch + 1            # restarted on the right time base
    assert win._psd is None and win._profile_dock._spec_status_lbl.text() == '—'
    assert '8107.4' in win._fs_lbl.text() and 'resampled' in win._fs_lbl.text()
    assert win._fs_lbl.toolTip() == BANNER

    sw.info_ready.emit(BANNER.replace('8107.419', '8107.455'))   # the next measurement
    assert win._stream.input_rate == 8107.455 and win._stream.epoch == epoch + 1


def test_a_new_connection_starts_from_the_nominal_rate_again(win, iv, monkeypatch):
    monkeypatch.setattr(iv.SerialWorker, 'start', lambda self: None)
    sw = iv.SerialWorker('FAKE', 115200, 2048.0)
    win._start_worker(sw)
    sw.info_ready.emit(BANNER)
    assert win._stream.input_rate != 8000.0
    win._start_worker(iv.SerialWorker('FAKE', 115200, 2048.0))   # e.g. older firmware: no line
    assert win._stream.input_rate == 8000.0 and 'configured' in win._fs_lbl.text()
    sw.info_ready.emit(BANNER)                                   # the old worker is ignored
    assert win._stream.input_rate == 8000.0


def test_sensor_rate_stream_shows_a_tone_at_its_true_frequency(win, iv, monkeypatch):
    monkeypatch.setattr(iv.SerialWorker, 'start', lambda self: None)
    sw = iv.SerialWorker('FAKE', 115200, 2048.0)
    win._start_worker(sw)
    sw.info_ready.emit(BANNER)
    pool = win._pool = RecordingPool(win)
    t = np.arange(int(8107.419 * 3)) / 8107.419
    x = np.repeat(np.sin(2 * np.pi * 2000.0 * t)[:, None], 3, axis=1).astype(np.float32)
    for i in range(0, len(x) - 79, 80):
        win._on_batch(x[i:i + 80], 0)
    assert pool.launched                                         # a window was taken
    y = win._stream.recent_raw(8000)[:, 2].astype(np.float64)
    spec = np.abs(np.fft.rfft(y * np.hanning(8000)))
    assert int(spec.argmax()) == 2000                            # 1 Hz bins: 2000 Hz, not 1973


# ── Ticket 16 ────────────────────────────────────────────────────────────────

def test_dead_code_is_gone(iv, win):
    for name in ('_update_fs', '_fs_history', '_fft_done'):
        assert not hasattr(win, name)
    assert 'full ring buffer' not in win._plot_len_edit.toolTip()


def test_time_plot_follows_the_stream(win, rng):
    batch = rng.normal(size=(80, 3)).astype(np.float32)
    for _ in range(10):
        win._on_batch(batch, 0)
    win._refresh_display()
    y = win._curves['Accel Z'].yData                 # as given, before display downsampling
    assert np.array_equal(y[-80:], batch[:, 2])
    win._on_batch(batch, 7)
    assert win._drop_lbl.text() == 'Drops: 7'
