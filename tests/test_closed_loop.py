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
from scipy.signal import butter, firwin, iirpeak, lfilter, lfilter_zi, welch

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


# ── A rig that runs out of drive ('1008 debug 2') ────────────────────────────

class ResonantRig:
    """A broadband path that rolls off above 400 Hz, plus a resonance at 85 Hz
    standing ~35 dB above it — the shape the correction curve in the field
    screenshot implies. Flattening it means cutting the drive ~30 dB at the
    resonance, so anything broadband in the drive (clipping distortion) lands
    there 30 dB hot."""

    def __init__(self, rng, gain):
        self.rng, self.gain = rng, gain
        self.lp = butter(1, 400.0, fs=AUDIO_FS)
        self.pk = iirpeak(85.0, 1.5, fs=AUDIO_FS)
        self.z_lp, self.z_pk = np.zeros(1), np.zeros(2)
        self.fir = firwin(193, 3400.0, fs=AUDIO_FS)
        self.fz  = np.zeros(len(self.fir) - 1)
        self.phase = 0
        self.clipped_samples = self.samples = 0

    def run(self, worker, n_blocks):
        blocks = []
        for _ in range(n_blocks):
            sig, clipped = worker.render(4096, self.rng)
            if clipped:
                worker.clip_detected.emit()          # as the audio callback does
            self.clipped_samples += int(np.sum(np.abs(sig) >= 1.0))
            self.samples += len(sig)
            blocks.append(sig.astype(np.float64))
        x = np.concatenate(blocks)
        a, self.z_lp = lfilter(*self.lp, x, zi=self.z_lp)
        b, self.z_pk = lfilter(*self.pk, x, zi=self.z_pk)
        y, self.fz = lfilter(self.fir, [1.0], self.gain * (a + 56.0 * b), zi=self.fz)
        dec = y[self.phase::DECIM]
        self.phase = (self.phase - len(y)) % DECIM
        return dec


def run_sequence(win, app, rig, steps, profile_level=0.001, slider=-20):
    """Run a whole sequence against `rig`. Returns per-second records and the
    8 kHz samples, one array per second."""
    d = win._profile_dock
    win._pool = SyncPool()
    d._audio_rate_combo.setCurrentText(str(AUDIO_FS))
    d._ctrl_enable_cb.setChecked(True)
    d._max_corr_spin.setValue(80)                    # room to cut a 35 dB resonance
    d._audio_slider.setValue(slider)
    m = d._bp_model
    m.setData(m.index(0, 1), str(profile_level)); m.setData(m.index(1, 1), str(profile_level))
    sm = d._seq_model
    while sm.rowCount() < len(steps):
        sm.add_row()
    for r, (dur, gain) in enumerate(steps):
        sm.setData(sm.index(r, 1), str(dur)); sm.setData(sm.index(r, 2), str(gain))
    d._start_test()
    d._timer.stop()
    worker = d._audio_worker
    pending, done, log, rec = np.empty(0), 0, [], []
    total = int(sum(dur for dur, _ in steps))
    for second in range(1, total + 1):
        n_blocks = int(round(second * AUDIO_FS / 4096.0)) - done
        done += n_blocks
        pending = np.concatenate([pending, rig.run(worker, n_blocks)])
        n = len(pending) // 80
        z, pending = pending[:n * 80], pending[n * 80:]
        rec.append(z)
        for b in z.reshape(-1, 80):
            batch = np.zeros((80, 3), dtype=np.float32)
            batch[:, 2] = b
            win._on_batch(batch, 0)
            app.processEvents()
        log.append(dict(
            out_dbfs=20 * np.log10(worker._output_gain),
            meas=float(d._grms_meas_lbl.text().split()[1]) if '—' not in d._grms_meas_lbl.text() else np.nan,
            limit=d._limit_lbl.text() if d._limit_lbl.isVisibleTo(d) else '',
            clip=d._audio_clip_lbl.text(),
            err=d._ctrl_err_lbl.text(),
        ))
        if second < total:
            d._tick()
    return log, rec


def resonance_vs_band_db(x):
    """How far the resonance stands above (+) or below (-) the rest of the band."""
    f, p = welch(x, fs=8000.0, window='hann', nperseg=8000, noverlap=4000)
    res  = np.mean(p[(f >= 70) & (f <= 100)])
    rest = np.mean(p[(f >= 300) & (f <= 1500)])
    return 10 * np.log10(res / rest)


STEPS = [(30, -12.0), (20, -8.0), (20, -6.0)]
RIG_SHORT = 0.18          # needs about +2 dBFS of clean drive at the last step


def test_rig_out_of_drive_stops_clean_and_says_so(win, app, rng):
    """New behaviour: stop at the ceiling, keep the shape, report the shortfall."""
    rig = ResonantRig(rng, RIG_SHORT)
    log, rec = run_sequence(win, app, rig, STEPS, slider=0)      # slider at max, as in the field

    assert max(r['out_dbfs'] for r in log) == pytest.approx(-12.0, abs=1e-6)
    assert rig.clipped_samples / rig.samples < 2e-4              # a stray peak, no more
    assert all(r['clip'] == '' for r in log)

    # At the limit from early in the sequence, and it says by how much.
    last = log[-1]
    assert 'DRIVE AT LIMIT' in last['limit']
    short = float(last['limit'].split('rig is ')[1].split(' dB')[0])
    assert short == pytest.approx(20 * np.log10(psd_grms([(20, 0.001), (2000, 0.001)], -6.0)
                                                / last['meas']), abs=0.5)
    assert 10.0 < short < 18.0
    # Steps cannot add anything from the ceiling — and the shortfall grows by the step.
    assert log[49]['meas'] == pytest.approx(log[69]['meas'], rel=0.05)
    short_2 = float(log[49]['limit'].split('rig is ')[1].split(' dB')[0])
    assert short - short_2 == pytest.approx(2.0, abs=0.5)

    # The shape is still under control: the resonance is flattened into the band.
    assert abs(resonance_vs_band_db(np.concatenate(rec[-10:]))) < 2.5       # measured +0.3…+1.2
    assert 'sat' not in last['err']
    assert win._ctl.corr_db.min() > -35.0                        # nowhere near the cut rail


def test_same_rig_driven_into_clipping_loses_the_shape(win, app, rng):
    """Old behaviour, by raising the ceiling to full scale: what '1008 debug 2'
    showed. More level, but the resonance cannot be held down."""
    win._max_drive_spin.setValue(0.0)
    rig = ResonantRig(rng, RIG_SHORT)
    log, rec = run_sequence(win, app, rig, STEPS, slider=0)

    assert log[-1]['out_dbfs'] == pytest.approx(0.0, abs=1e-6)   # pinned at full scale
    assert rig.clipped_samples / rig.samples > 0.15
    assert log[-1]['clip'] == 'CLIP'
    assert log[49]['meas'] == pytest.approx(log[69]['meas'], rel=0.05)   # steps add nothing here either
    # The resonance stands well above the rest of the band although the loop
    # has cut the drive there as far as it is allowed to.
    assert resonance_vs_band_db(np.concatenate(rec[-10:])) > 5.0            # measured +7.6…+8.2
    assert win._ctl.corr_db.min() == pytest.approx(-40.0)
    assert 'sat' in log[-1]['err']


def test_same_rig_with_the_amplifier_turned_up_tracks_every_step(win, app, rng):
    rig = ResonantRig(rng, RIG_SHORT * 10 ** (15 / 20))          # +15 dB of amplifier
    log, rec = run_sequence(win, app, rig, STEPS, slider=-30)
    for end_s, gain_db in ((30, -12.0), (50, -8.0), (70, -6.0)):
        want = psd_grms([(20, 0.001), (2000, 0.001)], gain_db)
        assert log[end_s - 1]['meas'] == pytest.approx(want, rel=0.05), (end_s, gain_db)
    assert all(r['limit'] == '' for r in log[20:])
    assert rig.clipped_samples / rig.samples < 2e-4
    assert abs(resonance_vs_band_db(np.concatenate(rec[-10:]))) < 2.5
