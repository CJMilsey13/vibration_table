"""SpectralController, SpecAssessor and the PSD helpers, through their interface.

Ticket numbers refer to issues/NN-*.md.
"""

import json
import math

import numpy as np
import pytest

from control import (
    CTRL_MAX_CUT_DB, LEVEL_MAX_DBFS_DEFAULT, LEVEL_MIN_DB, ResponseError,
    SpecAssessor, SpectralController, band_grms, band_mask, breakpoint_error,
    gaussian_clip, psd_grms, psd_interp_loglog,
)
from sim import (
    FREQS, PROFILE, START_DBFS, SimRig, controller, flat_plant_db, measure, on_target,
)

BAND = band_mask(FREQS, PROFILE)


def run_loop(ctl, rig, n_updates, gain_db=0.0, drive_active=True):
    outs = []
    for _ in range(n_updates):
        meas = rig.measure(ctl.corr_db, ctl.output_dbfs(gain_db))
        outs.append(ctl.update(meas, PROFILE, gain_db, drive_active))
    return outs


# ── Ticket 01: no integration without a drive ────────────────────────────────

def test_drive_off_integrates_nothing(rng):
    ctl = SpectralController(FREQS)
    ctl.reseed_level(-20.0)
    ambient = np.full(len(FREQS), 4.2e-9)           # sensor noise floor, g²/Hz
    for _ in range(30):                             # 60 s of loop-on, drive-off
        out = ctl.update(measure(ambient, rng), PROFILE, 0.0, drive_active=False)
        assert not out.acted
    assert ctl.level_db == -20.0
    assert not np.any(ctl.corr_db)
    assert out.common_db > 40.0                     # the error is still reported


def test_same_input_with_drive_on_does_integrate(rng):
    """The guard above is what holds the level — not a quiet error signal."""
    ctl = SpectralController(FREQS)
    ctl.reseed_level(-20.0)
    ambient = np.full(len(FREQS), 4.2e-9)
    ctl.update(measure(ambient, rng), PROFILE, 0.0, drive_active=True)
    assert ctl.level_db == pytest.approx(-14.0)     # one slew-limited step


# ── Ticket 07: the level servo is unbiased ───────────────────────────────────

def test_level_error_is_unbiased_on_target(rng):
    ctl = SpectralController(FREQS)
    commons = [ctl.update(on_target(rng), PROFILE, 0.0, False).common_db
               for _ in range(60)]
    assert abs(np.mean(commons)) < 0.1
    # ...whereas the mean of per-bin dB errors, which the servo used to null,
    # sits well away from zero on the very same windows.
    demand = psd_interp_loglog(FREQS, PROFILE)
    db_means = [float(np.mean(10 * np.log10(demand[BAND] / on_target(rng)[BAND])))
                for _ in range(20)]
    assert np.mean(db_means) > 0.5


def test_converged_rig_runs_at_demand_not_above_it(rng):
    rig = SimRig(np.full(len(FREQS), flat_plant_db(9.0)), rng)
    ctl = controller()
    run_loop(ctl, rig, 12)
    # True in-band power of the rig once settled, against demand.
    true   = rig.true_psd(ctl.corr_db, ctl.output_dbfs(0.0))
    demand = psd_interp_loglog(FREQS, PROFILE)
    ratio_db = 10 * math.log10(np.sum(true[BAND]) / np.sum(demand[BAND]))
    assert abs(ratio_db) < 0.3                      # was +0.8 dB hot


# ── Convergence against a simulated rig ──────────────────────────────────────

def test_cold_start_nine_db_low_converges(rng):
    """CLAUDE.md, 'Two loops': split level + shape settles in ~6 s with a
    peak correction of a few dB, where the single loop never did."""
    rig = SimRig(np.full(len(FREQS), flat_plant_db(9.0)), rng)
    ctl = controller()
    outs = run_loop(ctl, rig, 8)
    assert abs(outs[3].common_db) < 0.5             # 4th update = 6 s of data
    assert all(abs(o.common_db) < 0.5 for o in outs[3:])
    assert np.max(np.abs(ctl.corr_db)) < 5.0        # level error never reached the shape loop
    assert outs[-1].sat_frac == 0.0


def test_tilted_plant_is_flattened(rng):
    tilt = -12.0 * np.log10(np.maximum(FREQS, 20.0) / 20.0) / 2.0   # -12 dB over 20→2000 Hz
    rig  = SimRig(flat_plant_db(3.0) + tilt, rng)
    ctl = controller()
    outs = run_loop(ctl, rig, 15)
    assert outs[-1].err_rms_db < 1.5
    # The correction learned is the inverse of the plant's tilt (up to a
    # constant, which the normalisation removes).
    learned = ctl.corr_db[BAND] - np.mean(ctl.corr_db[BAND])
    wanted  = -(tilt[BAND] - np.mean(tilt[BAND]))
    assert np.sqrt(np.mean((learned - wanted) ** 2)) < 1.5


def test_error_readout_is_low_on_a_perfect_signal(rng):
    """Ticket 06: the old raw-bin RMS read 2.87 dB here, at the green limit."""
    ctl  = SpectralController(FREQS)
    errs = [ctl.update(on_target(rng), PROFILE, 0.0, False).err_rms_db
            for _ in range(20)]
    assert np.mean(errs) < 1.0


# ── Ticket 04: a sequence step is a feedforward level change ─────────────────

def test_gain_step_moves_output_by_the_step_and_needs_no_relearning(rng):
    rig = SimRig(np.full(len(FREQS), flat_plant_db(4.0)), rng)
    ctl = controller()
    run_loop(ctl, rig, 8, gain_db=-6.0)
    before = ctl.output_dbfs(-6.0)
    assert ctl.output_dbfs(0.0) - before == pytest.approx(6.0)
    out = run_loop(ctl, rig, 1, gain_db=0.0)[0]     # first window after the step
    assert abs(out.common_db) < 0.5


def test_drive_stops_at_the_ceiling_and_says_how_far_short(rng):
    """'1008 debug 2': the rig needs more than the DAC can cleanly give."""
    short = 7.0                                      # rig needs 7 dB more than the ceiling
    rig = SimRig(np.full(len(FREQS), flat_plant_db(short, at_dbfs=LEVEL_MAX_DBFS_DEFAULT)), rng)
    ctl = controller()
    outs = run_loop(ctl, rig, 14)
    assert ctl.output_dbfs(0.0) == LEVEL_MAX_DBFS_DEFAULT        # on the ceiling, not past it
    assert outs[-1].at_limit
    assert outs[-1].shortfall_db == pytest.approx(short, abs=0.3)   # what the amplifier must add
    assert not outs[0].at_limit                      # not while it still had room to climb
    # The shape loop is unaffected: it goes on flattening what the rig can give.
    assert np.max(np.abs(ctl.corr_db)) < 5.0 and outs[-1].sat_frac == 0.0


def test_a_step_up_from_the_ceiling_adds_nothing(rng):
    rig = SimRig(np.full(len(FREQS), flat_plant_db(2.0, at_dbfs=LEVEL_MAX_DBFS_DEFAULT)), rng)
    ctl = controller()
    run_loop(ctl, rig, 12, gain_db=-12.0)            # needs ceiling - 10 dB: fine
    assert not run_loop(ctl, rig, 1, gain_db=-12.0)[0].at_limit
    before = ctl.output_dbfs(-12.0)
    assert ctl.output_dbfs(-6.0) - before == pytest.approx(6.0)     # room for this step
    assert ctl.output_dbfs(0.0) == LEVEL_MAX_DBFS_DEFAULT           # ...but not for this one
    assert ctl.output_limited(0.0) and not ctl.output_limited(-6.0)
    out = run_loop(ctl, rig, 4, gain_db=0.0)[-1]
    assert out.at_limit and out.shortfall_db == pytest.approx(2.0, abs=0.3)


def test_ceiling_holds_at_every_gain_and_can_be_moved(rng):
    rig = SimRig(np.full(len(FREQS), flat_plant_db(40.0)), rng)   # hopeless rig
    ctl = controller()
    for gain_db in (-6.0, 0.0, 6.0):
        run_loop(ctl, rig, 12, gain_db=gain_db)
        assert ctl.output_dbfs(gain_db) == LEVEL_MAX_DBFS_DEFAULT
        assert ctl.level_db <= LEVEL_MAX_DBFS_DEFAULT - gain_db + 1e-9   # servo stopped at the rail
    assert ctl.level_db >= LEVEL_MIN_DB
    ctl.max_output_dbfs = -3.0                       # operator's choice, on the Settings tab
    run_loop(ctl, rig, 6, gain_db=0.0)
    assert ctl.output_dbfs(0.0) == -3.0


def test_gaussian_clip_table():
    """The figures quoted in control.py and on the Settings tab."""
    for dbfs, frac, sdr in ((0.0, 0.317, 9.7), (-6.0, 0.046, 19.8),
                            (-9.5, 0.0028, 33.7), (-12.0, 6.9e-5, 51.7)):
        f, s_ = gaussian_clip(dbfs)
        assert f == pytest.approx(frac, rel=0.03) and s_ == pytest.approx(sdr, abs=0.3)
    # ...and against a brute-force clip of real Gaussian samples.
    x = np.random.default_rng(0).normal(size=2_000_000)
    for dbfs in (0.0, -6.0):
        k = 10 ** (-dbfs / 20)
        y = np.clip(x, -k, k)
        alpha = np.dot(x, y) / np.dot(x, x)
        sdr = 10 * np.log10(alpha ** 2 * np.mean(x ** 2) / np.mean((y - alpha * x) ** 2))
        assert gaussian_clip(dbfs)[1] == pytest.approx(sdr, abs=0.2)
        assert gaussian_clip(dbfs)[0] == pytest.approx(np.mean(np.abs(x) > k), rel=0.02)


def test_reseed_level(rng):
    ctl = SpectralController(FREQS)
    ctl.reseed_level(-33.0)
    assert ctl.level_db == -33.0 and ctl.output_dbfs(-6.0) == -39.0
    ctl.reseed_level(+12.0)
    assert ctl.level_db == 0.0
    # The slider can sit above the ceiling; the DAC still does not go there.
    assert ctl.output_dbfs(0.0) == LEVEL_MAX_DBFS_DEFAULT and ctl.output_limited(0.0)
    assert ctl.output_dbfs(-20.0) == -20.0 and not ctl.output_limited(-20.0)


# ── Clamp and anti-windup (CLAUDE.md, 'Why the clamp is asymmetric') ─────────

def test_resonance_fed_by_the_whole_band_cannot_wind_the_cut(rng):
    # At the start level the resonance sits ~10 dB above demand whatever is driven there.
    rig = SimRig(np.full(len(FREQS), flat_plant_db(0.0)), rng,
                 coupling=0.5 * 10 ** (-(START_DBFS + 20.0) / 10))
    ctl = controller()
    ctl.max_boost_db = 200.0                        # boost ceiling must not matter
    outs = run_loop(ctl, rig, 40)
    assert ctl.corr_db.min() == pytest.approx(-CTRL_MAX_CUT_DB)   # on the rail, not past it
    assert outs[-1].sat_frac > 0.0                  # and the readout says so
    # The rest of the band is still being controlled, not starved.
    away = BAND & (np.abs(FREQS - 600.0) > 150.0)
    assert np.max(np.abs(ctl.corr_db[away])) < 10.0


def test_default_boost_limit_does_not_limit_the_cut(rng):
    """Max corr is a BOOST ceiling. A 30 dB resonance must still be cut flat
    with it at the default 20 dB."""
    resonance = 30.0 * np.exp(-0.5 * ((FREQS - 300.0) / 40.0) ** 2)
    rig = SimRig(flat_plant_db(3.0) + resonance, rng)
    ctl = controller()
    assert ctl.max_boost_db == 20.0 and ctl.limits_db == (-CTRL_MAX_CUT_DB, 20.0)
    outs = run_loop(ctl, rig, 20)
    assert ctl.corr_db.min() < -24.0                 # went well past -20
    assert ctl.corr_db.max() <= 20.0
    assert outs[-1].err_rms_db < 1.5 and outs[-1].sat_frac == 0.0


def test_correction_is_zero_outside_the_profile_band(rng):
    rig = SimRig(np.full(len(FREQS), flat_plant_db(9.0)), rng)
    ctl = controller()
    run_loop(ctl, rig, 6)
    assert not np.any(ctl.corr_db[~BAND])


def test_reset_returns_to_base_not_flat(rng, tmp_path):
    rig = SimRig(flat_plant_db(3.0) - 6.0 * (FREQS > 500), rng)
    ctl = controller()
    run_loop(ctl, rig, 10)
    ctl.save_response(tmp_path / 'r.json')
    base = ctl.base_db
    run_loop(ctl, SimRig(np.full(len(FREQS), flat_plant_db(3.0)), rng), 6)
    assert not np.allclose(ctl.corr_db, base)
    ctl.reset()
    assert np.array_equal(ctl.corr_db, base) and np.any(base)


# ── Ticket 03: saved speaker response ────────────────────────────────────────

def write_response(path, freqs, corr):
    path.write_text(json.dumps({'freqs_hz': list(map(float, freqs)),
                                'corr_db': list(map(float, corr))}))
    return path


def test_wound_up_response_is_refused(tmp_path):
    """Same shape as the file found on disk: -22…+120 dB, in-band mean 103."""
    corr = np.where(BAND, 103.0, 0.0)
    corr[FREQS > 1800] = 120.0
    corr[FREQS < 12] = -22.0
    ctl = SpectralController(FREQS)
    with pytest.raises(ResponseError, match='wound-up'):
        ctl.load_response(write_response(tmp_path / 'r.json', FREQS, corr))
    assert not np.any(ctl.corr_db) and not np.any(ctl.base_db)


def test_the_response_file_on_disk_is_refused():
    from pathlib import Path
    real = Path(__file__).resolve().parent.parent / 'speaker_response.json'
    if not real.exists():
        pytest.skip('no speaker_response.json in the repo')
    data = json.loads(real.read_text(encoding='utf-8'))
    if max(data['corr_db']) <= 20.0:
        pytest.skip('response file has been relearned')
    ctl = SpectralController(FREQS)
    with pytest.raises(ResponseError):
        ctl.load_response(real)


def test_sane_response_loads_clamped_and_tapered(tmp_path):
    f_saved = np.arange(20.0, 2001.0)
    c_saved = 8.0 * np.sin(np.linspace(0, 6, len(f_saved)))
    c_saved[:20] = 35.0                              # 1 % of bins past the +20 dB rail
    ctl = SpectralController(FREQS)
    ctl.load_response(write_response(tmp_path / 'r.json', f_saved, c_saved))
    corr = ctl.corr_db
    assert corr.max() == pytest.approx(20.0)         # clamped to the loop's rail
    assert not np.any(corr[(FREQS < 20) | (FREQS > 2000)])   # tapered, not edge-held
    assert np.array_equal(corr, ctl.base_db)
    mid = (FREQS > 100) & (FREQS < 1900)
    assert np.allclose(corr[mid], c_saved[(f_saved > 100) & (f_saved < 1900)], atol=1e-6)


def test_save_then_load_round_trips(rng, tmp_path):
    rig = SimRig(flat_plant_db(3.0) - 6.0 * (FREQS > 500), rng)
    ctl = controller()
    run_loop(ctl, rig, 10)
    ctl.save_response(tmp_path / 'r.json')
    other = SpectralController(FREQS)
    other.load_response(tmp_path / 'r.json')
    assert np.allclose(other.corr_db, ctl.corr_db, atol=1e-3)


@pytest.mark.parametrize('payload', [
    'not json',
    '{"freqs_hz": [1, 2, 3]}',
    '{"freqs_hz": [1, 2, 3], "corr_db": [0, 0]}',
    '{"freqs_hz": [1, 2, 2], "corr_db": [0, 0, 0]}',
    '{"freqs_hz": [0, 1, 2], "corr_db": [0, 0, 0]}',
    '{"freqs_hz": [1, 2, 3], "corr_db": [0, NaN, 0]}',
    '{"freqs_hz": [1], "corr_db": [0]}',
    '{"freqs_hz": "abc", "corr_db": "abc"}',
])
def test_malformed_response_is_refused(tmp_path, payload):
    path = tmp_path / 'r.json'
    path.write_text(payload)
    ctl = SpectralController(FREQS)
    with pytest.raises(ResponseError):
        ctl.load_response(path)
    assert not np.any(ctl.corr_db)


def test_missing_response_file_is_refused(tmp_path):
    with pytest.raises(ResponseError):
        SpectralController(FREQS).load_response(tmp_path / 'absent.json')


# ── Ticket 06: in-spec status ────────────────────────────────────────────────

def verdict_after(rng, n_windows, n_avg=8, **kw):
    a = SpecAssessor(FREQS, n_avg)
    for _ in range(n_windows):
        a.add(on_target(rng, **kw), PROFILE, kw.get('gain_db', 0.0))
    return a.status(PROFILE)


def test_perfect_signal_reads_in_spec(rng):
    verdicts = [verdict_after(rng, 8).verdict for _ in range(40)]
    assert verdicts.count('IN SPEC') >= 38           # ≥ 95 %; was ABORT 100 %


def test_single_window_would_fail_which_is_why_it_gives_no_verdict(rng):
    s = verdict_after(rng, 1)
    assert s.verdict == 'AVERAGING' and not s.settled
    assert s.alarm_frac > 0.05                       # the scatter alone exceeds the limit
    assert verdict_after(rng, 1, n_avg=1).verdict == 'ABORT'   # what N=1 gives you


def test_real_excursions_are_still_caught(rng):
    assert verdict_after(rng, 8, offset_db=+8.0).verdict == 'ABORT'
    assert verdict_after(rng, 8, offset_db=-8.0).verdict == 'ABORT'
    # Between the two limits. With ~1 dB of scatter left at N=8 the ALARM-only
    # zone is narrow: much past 4 dB and enough bins cross ±6 dB to abort.
    assert verdict_after(rng, 8, offset_db=+3.5).verdict == 'ALARM'
    assert verdict_after(rng, 8, offset_db=-3.5).verdict == 'ALARM'


def test_gain_steps_do_not_invalidate_the_average(rng):
    a = SpecAssessor(FREQS, 8)
    for gain_db in (-6.0,) * 4 + (0.0,) * 4:         # on target at both steps
        a.add(on_target(rng, gain_db=gain_db), PROFILE, gain_db)
    assert a.status(PROFILE).verdict == 'IN SPEC'


def test_averaging_setting(rng):
    a = SpecAssessor(FREQS, 8)
    assert a.status(PROFILE) is None
    for _ in range(8):
        a.add(on_target(rng), PROFILE, 0.0)
    a.n_avg = 4                                      # keeps the most recent windows
    s = a.status(PROFILE)
    assert (s.n_windows, s.n_avg, s.settled) == (4, 4, True)
    a.n_avg = 16
    assert not a.status(PROFILE).settled
    a.n_avg = 0
    assert a.n_avg == 1
    a.n_avg = 10_000
    assert a.n_avg == 32
    a.reset()
    assert a.status(PROFILE) is None


# ── Tickets 10 and 11: PSD helpers ───────────────────────────────────────────

def test_band_grms_matches_demand_on_target(rng):
    meas = np.mean([on_target(rng) for _ in range(10)], axis=0)
    assert band_grms(FREQS, meas, PROFILE) == pytest.approx(psd_grms(PROFILE), rel=0.02)


def test_band_grms_ignores_out_of_band_energy(rng):
    meas = on_target(rng)
    loud = meas.copy()
    loud[~BAND] = 1.0
    assert band_grms(FREQS, loud, PROFILE) == band_grms(FREQS, meas, PROFILE)


def test_psd_grms_reference_values():
    assert psd_grms(PROFILE) == pytest.approx(math.sqrt(0.0005 * 1980.0))
    assert psd_grms(PROFILE, -6.0) == pytest.approx(
        math.sqrt(0.0005 * 1980.0) * 10 ** (-6 / 20))
    # -3 dB/octave slope (m = -1) takes the logarithmic branch.
    assert psd_grms([(10.0, 1.0), (100.0, 0.1)]) == pytest.approx(
        math.sqrt(10.0 * math.log(10.0)))


def test_duplicate_breakpoint_frequency_does_not_raise():
    assert psd_grms([(20.0, 1e-3), (20.0, 1e-3), (2000.0, 5e-4)]) > 0.0
    assert psd_grms([(20.0, 1e-3), (20.0, 1e-3)]) == 0.0


def test_breakpoint_error():
    assert breakpoint_error(PROFILE) is None
    assert 'distinct' in breakpoint_error([(20.0, 1e-3), (20.0, 1e-3), (2000.0, 5e-4)])
    assert breakpoint_error([(2000.0, 1e-3), (20.0, 1e-3)]) is not None
    assert breakpoint_error([(20.0, 0.0), (2000.0, 1e-3)]) is not None
    assert breakpoint_error([(-5.0, 1e-3), (2000.0, 1e-3)]) is not None
    assert breakpoint_error([(20.0, float('nan')), (2000.0, 1e-3)]) is not None
