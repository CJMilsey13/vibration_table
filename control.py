"""
Closed-loop spectral control — level servo, per-bin shape loop, in-spec status
and speaker-response persistence. Qt-free, so the control law can be run
against a simulated plant.

Everything here works on one frequency grid (the Welch bins inside
PSD_FMIN…PSD_FMAX) and on ONE measured PSD: the mean across the selected
control channels. The Grms readout, the in-spec status and the loop all read
that same estimate, so they cannot disagree with each other.
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
from scipy.ndimage import gaussian_filter1d

from stream import CTRL_WINDOW_S, SAMPLE_RATE, SPEC_N

Breakpoints = Sequence[tuple[float, float]]   # (freq_hz, level g²/Hz), ascending

CTRL_DEADBAND_DB  = 0.5    # ≈1σ of the smoothed estimate; soft-thresholded
CTRL_MAX_STEP_DB  = 6.0    # per-update slew limit on the correction
CTRL_SMOOTH_BINS  = 5.0    # gaussian sigma across frequency, in bins

# The clamp is ASYMMETRIC on purpose. Boosting a bin is legitimate whenever the
# rig is simply inefficient there. Cutting is different: once a bin is ~30 dB
# down it is effectively switched off, so if the measured level there has not
# fallen, that energy is not coming from the drive at that frequency — it is the
# structure ringing at resonance, or harmonics of some other bin. A resonant
# plant is not diagonal, and no amount of further cutting nulls a cross-coupled
# term; it only burns dynamic range and starves the bins that do respond.
CTRL_MAX_CUT_DB   = 40.0   # deep enough for a genuine high-Q resonance, while
                           # keeping boost+cut spread inside the DAC's ~96 dB

# Slew is asymmetric. Moving AWAY from zero is limited to CTRL_MAX_STEP_DB so one
# bad window cannot slam a bin to an extreme. Moving TOWARD zero is returning to
# neutral — inherently safe — so it is allowed to unwind a full rail within
# CTRL_RECOVERY_S. One control update is the floor: the loop cannot react faster
# than it can measure.
CTRL_RECOVERY_S    = 2.0
CTRL_MAX_UNWIND_DB = CTRL_MAX_CUT_DB * CTRL_WINDOW_S / CTRL_RECOVERY_S

# Overall level is a SEPARATE loop from spectral shape. The synthesised block is
# normalised to unit RMS, so a common-mode (uniform) per-bin correction produces
# exactly 0.00 dB of output change — it is divided straight back out. Only this
# scalar can move the level. Splitting the error into common-mode (here) and
# zero-mean (per-bin) also stops the shape loop winding every bin to the clamp
# chasing a level deficit it structurally cannot fix.
CTRL_LEVEL_MAX_STEP_DB = 6.0    # real physical level change — worth slew-limiting
LEVEL_MIN_DB           = -80.0
LEVEL_DEFAULT_DB       = -20.0  # safe start

# Tolerance bands drawn around the demand profile (dB power), the usual way a
# random-vibration run is judged in or out of spec.
TOL_ALARM_DB = 3.0
TOL_ABORT_DB = 6.0
SPEC_ALARM_FRAC = 0.05   # share of in-band bins allowed outside ±TOL_ALARM_DB
SPEC_ABORT_FRAC = 0.02   # share of in-band bins allowed outside ±TOL_ABORT_DB

# A single 2 s control window has ~3 Welch averages, so each bin scatters by
# ~2.9 dB on its own — a perfect signal would read ABORT. The in-spec verdict is
# therefore taken on a running average of the last N control windows.
SPEC_AVG_DEFAULT = 8
SPEC_AVG_MAX     = 32

# A saved response with more than this share of bins outside the loop's own
# clamp is not a transfer function, it is a wound-up integrator. Refuse it.
RESPONSE_MAX_CLAMPED_FRAC = 0.05


# ── PSD profile helpers ───────────────────────────────────────────────────────

def psd_interp_loglog(
    freqs: np.ndarray,
    breakpoints: Breakpoints,
    gain_db: float = 0.0,
) -> np.ndarray:
    """Log-log interpolate PSD breakpoints onto freqs; apply gain_db (power scale)."""
    if len(breakpoints) < 2:
        return np.full(len(freqs), 1e-30)
    bf = np.array([b[0] for b in breakpoints], dtype=np.float64)
    bp = np.array([b[1] for b in breakpoints], dtype=np.float64)
    lf = np.clip(np.log10(freqs), np.log10(bf[0]), np.log10(bf[-1]))
    lp = np.interp(lf, np.log10(bf), np.log10(bp))
    return (10.0 ** lp) * (10.0 ** (gain_db / 10.0))


def psd_grms(breakpoints: Breakpoints, gain_db: float = 0.0) -> float:
    """Integrate log-log PSD profile analytically → Grms."""
    if len(breakpoints) < 2:
        return 0.0
    gain    = 10.0 ** (gain_db / 10.0)
    grms_sq = 0.0
    for i in range(len(breakpoints) - 1):
        f1, p1 = breakpoints[i][0],   breakpoints[i][1]   * gain
        f2, p2 = breakpoints[i+1][0], breakpoints[i+1][1] * gain
        if f2 <= f1:
            continue    # zero-width segment contributes no area
        m = math.log10(p2 / p1) / math.log10(f2 / f1)
        if abs(m + 1.0) < 1e-9:
            grms_sq += p1 * f1 * math.log(f2 / f1)
        else:
            grms_sq += p1 / (m + 1.0) * (f2**(m+1.0) - f1**(m+1.0)) / (f1**m)
    return math.sqrt(max(grms_sq, 0.0))


def breakpoint_error(breakpoints: Breakpoints) -> Optional[str]:
    """Why this table cannot be used as a profile, or None if it can."""
    last_f = 0.0
    for f, p in breakpoints:
        if not (math.isfinite(f) and math.isfinite(p)) or f <= 0.0 or p <= 0.0:
            return 'frequency and level must be positive numbers'
        if f <= last_f:
            return f'frequencies must be distinct ({f:g} Hz appears twice)'
        last_f = f
    return None


def band_mask(freqs: np.ndarray, breakpoints: Breakpoints) -> np.ndarray:
    """Bins inside the profile band — the only place the loop acts or is judged."""
    if len(breakpoints) < 2:
        return np.zeros(len(freqs), dtype=bool)
    return (freqs >= breakpoints[0][0]) & (freqs <= breakpoints[-1][0])


def band_grms(freqs: np.ndarray, meas_psd: np.ndarray,
              breakpoints: Breakpoints) -> float:
    """Measured Grms over the profile band — the span psd_grms integrates."""
    band = band_mask(freqs, breakpoints)
    if not band.any():
        return 0.0
    df = float(freqs[1] - freqs[0])
    return math.sqrt(max(float(np.sum(meas_psd[band])) * df, 0.0))


# ── In-spec status ────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class SpecStatus:
    alarm_frac: float   # share of in-band bins outside ±TOL_ALARM_DB
    abort_frac: float   # share of in-band bins outside ±TOL_ABORT_DB
    n_windows:  int     # control windows in the average so far
    n_avg:      int     # windows wanted

    @property
    def settled(self) -> bool:
        """Enough averages for a verdict. Before this the scatter alone fails it."""
        return self.n_windows >= self.n_avg

    @property
    def verdict(self) -> str:
        if not self.settled:
            return 'AVERAGING'
        if self.abort_frac > SPEC_ABORT_FRAC:
            return 'ABORT'
        if self.alarm_frac > SPEC_ALARM_FRAC:
            return 'ALARM'
        return 'IN SPEC'


class SpecAssessor:
    """Judges the rig against tolerance on a running average of control windows.

    Each window is stored as measured/demand, so a sequence gain step — which
    moves demand and drive together — does not invalidate the windows already
    held. Feed it non-overlapping control windows only. Display only: nothing
    here is ever fed back into the loop.
    """

    def __init__(self, freqs: np.ndarray, n_avg: int = SPEC_AVG_DEFAULT) -> None:
        self._freqs  = np.asarray(freqs, dtype=np.float64)
        self._ratios: deque[np.ndarray] = deque(maxlen=self._clamp(n_avg))

    @staticmethod
    def _clamp(n_avg: int) -> int:
        return max(1, min(SPEC_AVG_MAX, int(n_avg)))

    @property
    def n_avg(self) -> int:
        return self._ratios.maxlen or 1

    @n_avg.setter
    def n_avg(self, n_avg: int) -> None:
        # Keeps the most recent windows when shrinking.
        self._ratios = deque(self._ratios, maxlen=self._clamp(n_avg))

    def reset(self) -> None:
        self._ratios.clear()

    def add(self, meas_psd: np.ndarray, breakpoints: Breakpoints,
            gain_db: float) -> None:
        demand = psd_interp_loglog(self._freqs, breakpoints, gain_db)
        self._ratios.append(np.maximum(meas_psd, 1e-30) / demand)

    def status(self, breakpoints: Breakpoints) -> Optional[SpecStatus]:
        band = band_mask(self._freqs, breakpoints)
        if not self._ratios or not band.any():
            return None
        # Linear average of power, then dB — averaging dB would bias it low.
        dev_db = np.abs(10.0 * np.log10(np.mean(self._ratios, axis=0)[band]))
        return SpecStatus(
            alarm_frac=float(np.mean(dev_db > TOL_ALARM_DB)),
            abort_frac=float(np.mean(dev_db > TOL_ABORT_DB)),
            n_windows=len(self._ratios),
            n_avg=self.n_avg,
        )


# ── Controller ────────────────────────────────────────────────────────────────

class ResponseError(ValueError):
    """A saved speaker response that must not be applied."""


@dataclass(frozen=True)
class ControlUpdate:
    acted:      bool         # False → nothing was integrated (drive inactive / no band)
    level_db:   float        # level servo state after this update
    corr_db:    np.ndarray   # per-bin correction after this update (copy)
    band:       np.ndarray   # bins inside the profile band
    common_db:  float        # level error: demand/measured in-band power, dB
    err_rms_db: float        # level error ⊕ smoothed shape error — what the loop acts on
    sat_frac:   float        # share of in-band bins on either rail


class SpectralController:
    """Two loops on every fresh control window: a scalar level servo and a
    per-bin integral shape loop.

    Call `update` exactly once per NON-OVERLAPPING measurement window. That is
    the single most important invariant: the plant is memoryless in dB, so a
    plain integrator is deadbeat at gain 1 and stable below 2 — but only if
    each update sees data that already reflects the previous one.

    `level_db` is referred to the 0 dB profile. What reaches the DAC is
    `output_dbfs(gain_db)` = level + sequence gain, so a sequence step moves the
    drive by exactly the step (feedforward) and the servo only trims the rest.
    """

    def __init__(self, freqs: np.ndarray) -> None:
        self._freqs    = np.asarray(freqs, dtype=np.float64)
        # _base is the saved rig response that reset() falls back to; _corr is
        # what is actually applied. dB power, 0 = no correction.
        self._base     = np.zeros(len(self._freqs), dtype=np.float64)
        self._corr     = np.zeros(len(self._freqs), dtype=np.float64)
        self._level_db = LEVEL_DEFAULT_DB
        self.loop_gain    = 0.5    # fraction of the dB error applied per update
        self.max_boost_db = 20.0   # boost ceiling; cut is capped at CTRL_MAX_CUT_DB

    # ── Level ─────────────────────────────────────────────────────────────────

    @property
    def level_db(self) -> float:
        return self._level_db

    def reseed_level(self, level_db: float) -> None:
        """Restart the level servo from an operator-chosen level."""
        self._level_db = max(LEVEL_MIN_DB, min(0.0, float(level_db)))

    def output_dbfs(self, gain_db: float) -> float:
        """Drive level for a step at gain_db. Never above full scale."""
        return max(LEVEL_MIN_DB, min(0.0, self._level_db + gain_db))

    # ── Correction ────────────────────────────────────────────────────────────

    @property
    def corr_db(self) -> np.ndarray:
        return self._corr.copy()

    @property
    def base_db(self) -> np.ndarray:
        return self._base.copy()

    @property
    def limits_db(self) -> tuple[float, float]:
        """(cut, boost) rails. Asymmetric — see CTRL_MAX_CUT_DB."""
        return -min(self.max_boost_db, CTRL_MAX_CUT_DB), self.max_boost_db

    def reset(self) -> None:
        """Discard what the loop has learned. Falls back to the saved speaker
        response, not to flat — the rig's transfer function doesn't change
        between tests."""
        self._corr = self._base.copy()

    def update(self, meas_psd: np.ndarray, breakpoints: Breakpoints,
               gain_db: float, drive_active: bool) -> ControlUpdate:
        """One control update from one fresh, non-overlapping window.

        drive_active must be False whenever the drive is not actually playing
        or is being held (paused). The error is then still reported, but
        neither loop integrates: with no drive the measurement is ambient
        noise, and integrating it winds the level to full scale.
        """
        band = band_mask(self._freqs, breakpoints)
        if not band.any():
            return ControlUpdate(False, self._level_db, self._corr.copy(),
                                 band, 0.0, 0.0, 0.0)

        demand = psd_interp_loglog(self._freqs, breakpoints, gain_db)
        meas   = np.maximum(meas_psd, 1e-30)

        # dB error: positive → we're below demand → need to drive harder
        err_db = 10.0 * np.log10(demand) - 10.0 * np.log10(meas)
        err_db[~band] = 0.0

        # Split the error. The common-mode part is a pure level deficit; the
        # per-bin loop is blind to it (unit-RMS normalisation cancels any
        # uniform correction exactly), so it goes to the level servo. It is
        # taken from LINEAR in-band power: the mean of per-bin dB errors is
        # biased by the low-averaging PSD estimate (+0.78 dB on a perfectly
        # on-target signal), and a servo nulling that runs the rig hot.
        common = 10.0 * math.log10(
            float(np.sum(demand[band])) / float(np.sum(meas[band])))
        # What remains is made zero-mean in dB and is genuinely about spectral
        # shape — which is also the only part that survives the normalisation.
        err_shape = err_db - float(np.mean(err_db[band]))
        err_shape[~band] = 0.0

        # Smooth error across frequency — one noisy bin must not swing its
        # own correction independently of its neighbours.
        err_smooth = gaussian_filter1d(err_shape, sigma=CTRL_SMOOTH_BINS)

        lo, hi = self.limits_db
        err_rms = math.sqrt(common ** 2 + float(np.mean(err_smooth[band] ** 2)))

        if not drive_active:
            return ControlUpdate(False, self._level_db, self._corr.copy(), band,
                                 common, err_rms, self._sat_frac(band, lo, hi))

        # Level servo. Slew-limited because it is a real physical level change.
        # The ceiling keeps level + gain at or below full scale, and stops the
        # servo integrating into a rail it is already on.
        self._level_db = max(LEVEL_MIN_DB, min(
            -gain_db,
            self._level_db + float(np.clip(
                common, -CTRL_LEVEL_MAX_STEP_DB, CTRL_LEVEL_MAX_STEP_DB))))

        # Soft deadband: shrink toward zero rather than hard-gating, so the
        # correction stops random-walking on measurement noise once
        # converged, without chattering at the threshold.
        err_eff = np.sign(err_smooth) * np.maximum(
            np.abs(err_smooth) - CTRL_DEADBAND_DB, 0.0)

        # Integral update. The plant is memoryless in dB
        # (measured_dB = drive_dB + plant_dB), so error decays as
        # (1 - gain)^n — deadbeat at gain 1, stable below 2. Valid only
        # because each update sees a fresh, non-overlapping window.
        #
        # Asymmetric response. Growing |correction| uses the user's gain and
        # the tight slew limit, because that is where stability is at stake.
        # Shrinking it is a return to neutral — the worst case is landing at
        # zero correction, i.e. driving the raw profile — so it runs at unity
        # gain and may unwind a full rail in a single update.
        raw     = self.loop_gain * err_eff
        toward  = (raw * self._corr) < 0.0
        raw     = np.where(toward, err_eff, raw)
        lim     = np.where(toward, CTRL_MAX_UNWIND_DB, CTRL_MAX_STEP_DB)
        step    = np.clip(raw, -lim, lim)
        # ...but never fling a bin through zero out the other side; land on it.
        over        = toward & (np.abs(step) > np.abs(self._corr))
        step[over]  = -self._corr[over]

        # Anti-windup by conditional integration: never accumulate further
        # into a rail we are already sitting on. Without this a bin the rig
        # cannot reach keeps integrating, so when conditions change it has to
        # unwind through tens of dB before the drive responds at all — which
        # is what stalls recovery and looks like the loop being stuck.
        at_hi = (self._corr >= hi - 1e-9) & (step > 0.0)
        at_lo = (self._corr <= lo + 1e-9) & (step < 0.0)
        step[at_hi | at_lo] = 0.0

        self._corr += step
        np.clip(self._corr, lo, hi, out=self._corr)
        self._corr[~band] = 0.0

        return ControlUpdate(True, self._level_db, self._corr.copy(), band,
                             common, err_rms, self._sat_frac(band, lo, hi))

    def _sat_frac(self, band: np.ndarray, lo: float, hi: float) -> float:
        hc = self._corr[band]
        return float(np.mean((hc >= hi - 1e-6) | (hc <= lo + 1e-6)))

    # ── Speaker response persistence ──────────────────────────────────────────

    def save_response(self, path: Path) -> None:
        """Store the current correction as this rig's speaker response."""
        path.write_text(json.dumps({
            'note':     'Learned inverse response of the drive chain '
                        '(amp + speaker/shaker + fixture). dB power.',
            'fs_hz':    SAMPLE_RATE,
            'spec_n':   SPEC_N,
            'freqs_hz': [round(float(f), 3) for f in self._freqs],
            'corr_db':  [round(float(c), 3) for c in self._corr],
        }, indent=1), encoding='utf-8')
        self._base = self._corr.copy()

    def load_response(self, path: Path) -> None:
        """Adopt a saved response as both the base and the applied correction.

        Raises ResponseError, leaving the controller untouched, if the file is
        unreadable, malformed, or mostly outside the loop's own clamp. A curve
        that passes is still clamped, so loading can never apply more than the
        loop itself would be allowed to.
        """
        try:
            data    = json.loads(path.read_text(encoding='utf-8'))
            f_saved = np.asarray(data['freqs_hz'], dtype=np.float64)
            c_saved = np.asarray(data['corr_db'],  dtype=np.float64)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise ResponseError(f'could not read file: {exc}') from exc
        if f_saved.ndim != 1 or len(f_saved) < 2 or f_saved.shape != c_saved.shape:
            raise ResponseError('freqs_hz and corr_db must be equal-length lists')
        if not (np.all(np.isfinite(f_saved)) and np.all(np.isfinite(c_saved))):
            raise ResponseError('file contains non-finite values')
        if f_saved[0] <= 0.0 or np.any(np.diff(f_saved) <= 0.0):
            raise ResponseError('freqs_hz must be positive and increasing')

        # Re-grid onto the current analysis bins; taper to 0 dB outside the
        # saved span so an old/narrower file can't inject edge-held boost.
        curve = np.interp(
            np.log10(self._freqs), np.log10(f_saved), c_saved,
            left=0.0, right=0.0,
        )
        lo, hi  = self.limits_db
        in_span = (self._freqs >= f_saved[0]) & (self._freqs <= f_saved[-1])
        outside = (curve < lo - 1e-6) | (curve > hi + 1e-6)
        frac    = float(np.mean(outside[in_span])) if in_span.any() else 0.0
        if frac > RESPONSE_MAX_CLAMPED_FRAC:
            raise ResponseError(
                f'{frac * 100:.0f}% of the curve is outside the loop limits '
                f'({lo:+.0f}…{hi:+.0f} dB; file spans {c_saved.min():+.0f}…'
                f'{c_saved.max():+.0f} dB). That is a wound-up correction, not '
                'a speaker response — relearn and save it again.')

        self._base = np.clip(curve, lo, hi)
        self._corr = self._base.copy()
