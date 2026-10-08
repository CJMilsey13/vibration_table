"""End to end: the real window running a whole test sequence against a
simulated rig.

Nothing is stubbed between the audio worker and the control loop:

    AudioOutputWorker.render  → 48 kHz drive blocks (real synthesis)
    simulated rig             → a 300 Hz first-order roll-off, ≈17 dB of tilt
    decimate to 8 kHz         → MainWindow._on_batch (real stream, real Welch)
    control loop              → correction + level back into the worker

This is the second, independent check on the module tests: it does not use
SimRig or the PSD-domain plant model at all.
"""

import numpy as np
import pytest
from scipy.signal import butter, firwin, lfilter, lfilter_zi, welch

from control import psd_grms

PROFILE  = [(20.0, 0.0005), (2000.0, 0.0005)]
AUDIO_FS = 48000
DECIM    = 6                      # 48 kHz → 8 kHz exactly
RIG_GAIN = 30.0                   # g per full-scale DAC unit


class SyncPool:
    """Runs each Welch inline so the simulation is deterministic."""

    def start(self, runnable):
        runnable.run()

    def waitForDone(self, *_):
        return True


class Rig:
    def __init__(self, rng):
        self.rng   = rng
        self.pb, self.pa = butter(1, 300.0, fs=AUDIO_FS)          # the "speaker"
        self.pz    = lfilter_zi(self.pb, self.pa) * 0.0
        self.fir   = firwin(193, 3400.0, fs=AUDIO_FS)             # anti-alias
        self.fz    = np.zeros(len(self.fir) - 1)
        self.phase = 0
        self.out   = np.empty(0)
        self.clipped = 0

    def run(self, worker, n_blocks):
        """Render n_blocks of drive and return what the accelerometer saw."""
        blocks = []
        for _ in range(n_blocks):
            sig, clipped = worker.render(4096, self.rng)
            self.clipped += clipped
            blocks.append(sig.astype(np.float64))
        x = np.concatenate(blocks)
        y, self.pz = lfilter(self.pb, self.pa, RIG_GAIN * x, zi=self.pz)
        y, self.fz = lfilter(self.fir, [1.0], y, zi=self.fz)
        dec = y[self.phase::DECIM]
        self.phase = (self.phase - len(y)) % DECIM
        return dec


def grms_and_psd(x):
    f, p = welch(x, fs=8000.0, window='hann', nperseg=8000, noverlap=4000)
    band = (f >= 20) & (f <= 2000)
    return float(np.sqrt(np.sum(p[band]))), f[band], p[band]


def test_whole_sequence_closed_loop(win, app, rng):
    d = win._profile_dock
    win._pool = SyncPool()
    d._audio_rate_combo.setCurrentText(str(AUDIO_FS))
    d._ctrl_enable_cb.setChecked(True)
    win._spec_avg_spin.setValue(4)                   # verdict after 8 s

    m = d._seq_model
    m.setData(m.index(0, 1), '30'); m.setData(m.index(0, 2), '-6')
    m.setData(m.index(1, 1), '30'); m.setData(m.index(1, 2), '0')

    d._start_test()
    d._timer.stop()                                  # this test is the clock
    worker = d._audio_worker
    assert worker is not None and worker._fs == AUDIO_FS

    rig      = Rig(rng)
    pending  = np.empty(0)
    recorded = []                                    # every 8 kHz sample, per second
    levels   = []                                    # DAC level at the end of each second
    status   = []

    blocks_per_s = AUDIO_FS / 4096.0
    done_blocks  = 0
    for second in range(1, 61):
        n_blocks = int(round(second * blocks_per_s)) - done_blocks
        done_blocks += n_blocks
        pending = np.concatenate([pending, rig.run(worker, n_blocks)])
        n_batches = len(pending) // 80
        z = pending[:n_batches * 80]
        pending = pending[n_batches * 80:]
        recorded.append(z)
        for b in z.reshape(-1, 80):
            batch = np.empty((80, 3), dtype=np.float32)
            batch[:, 0] = 1e-3 * rng.normal(size=80)
            batch[:, 1] = 1e-3 * rng.normal(size=80)
            batch[:, 2] = b
            win._on_batch(batch, 0)
            app.processEvents()                      # deliver the queued Welch result
        levels.append(20 * np.log10(worker._output_gain))
        status.append(d._spec_status_lbl.text().split('  ')[0])
        if second < 60:
            d._tick()
        if second == 30:
            # The step's gain reached the DAC the instant the step began.
            assert 20 * np.log10(worker._output_gain) - levels[-1] == pytest.approx(6.0, abs=1e-6)

    # ── End of each step: on demand, in spec ─────────────────────────────────
    for end_s, gain_db in ((30, -6.0), (60, 0.0)):
        x = np.concatenate(recorded[end_s - 10:end_s])       # last 10 s of the step
        grms, f, p = grms_and_psd(x)
        want = psd_grms(PROFILE, gain_db)
        assert grms == pytest.approx(want, rel=0.03), (end_s, grms, want)
        # In-band PSD against demand, on an estimate with ~19 averages.
        dev_db = 10 * np.log10(p / (0.0005 * 10 ** (gain_db / 10)))
        assert np.mean(np.abs(dev_db) > 3.0) < 0.05, end_s
        assert status[end_s - 1] == 'IN SPEC', (end_s, status)

    # ── The loop did real work to get there ──────────────────────────────────
    corr = win._ctl.corr_db
    freqs = win._plot_freqs
    band  = (freqs >= 20) & (freqs <= 2000)
    lo, hi = corr[(freqs > 40) & (freqs < 80)].mean(), corr[(freqs > 1500) & (freqs < 1900)].mean()
    assert hi - lo == pytest.approx(10 * np.log10((1 + (1700 / 300) ** 2) / (1 + (60 / 300) ** 2)), abs=2.5)
    assert not np.any(corr[~band])
    assert levels[29] > -26.0 + 1.0                  # level servo raised the drive from its start
    # The step is applied once. A control window straddling the step reads the
    # old level against the new demand and adds the step a second time — seen
    # here as a +5 dB overshoot and clipping before the control window was
    # made to restart on every drive change.
    after_step = np.asarray(levels[30:60])
    assert np.all(np.abs(after_step - (levels[29] + 6.0)) < 1.0), after_step
    assert rig.clipped == 0
    assert 'sat' not in d._ctrl_err_lbl.text()

    # ── Grms timeline recorded by the app itself ─────────────────────────────
    t = np.asarray(win._grms_t); g = np.asarray(win._grms_v)
    for lo_t, hi_t, gain_db in ((12, 30, -6.0), (42, 60, 0.0)):
        sel = (t >= lo_t) & (t < hi_t)
        assert sel.sum() >= 6
        assert np.all(np.abs(20 * np.log10(g[sel] / psd_grms(PROFILE, gain_db))) < 1.0)

    # ── Completion stops the drive ───────────────────────────────────────────
    d._tick()
    assert not d.is_running and not d.drive_running
    level = win._ctl.level_db
    for _ in range(300):                             # 3 s more data with the loop still ticked
        win._on_batch(np.zeros((80, 3), dtype=np.float32), 0)
        app.processEvents()
    assert win._ctl.level_db == level
