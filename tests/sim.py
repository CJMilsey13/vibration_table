"""Simulated rig for the control tests.

Every measurement is real Gaussian noise put through the same Welch settings as
the application, so the estimator scatter the loop has to live with is the real
thing, not a model of it.
"""

from __future__ import annotations

import numpy as np
from scipy.signal import welch

from control import SpectralController, psd_interp_loglog
from stream import CTRL_SAMPLES, PSD_FMAX, PSD_FMIN, SAMPLE_RATE, SPEC_N

FULL_FREQS = np.fft.rfftfreq(SPEC_N, 1.0 / SAMPLE_RATE)
MASK       = (FULL_FREQS >= PSD_FMIN) & (FULL_FREQS <= PSD_FMAX)
FREQS      = FULL_FREQS[MASK]                 # the control grid, 1 Hz bins

PROFILE = [(20.0, 0.0005), (2000.0, 0.0005)]  # the application default


def noise(true_psd: np.ndarray, rng: np.random.Generator,
          n: int = CTRL_SAMPLES) -> np.ndarray:
    """n samples of Gaussian noise whose one-sided PSD is true_psd (on FREQS)."""
    fr  = np.fft.rfftfreq(n, 1.0 / SAMPLE_RATE)
    psd = np.interp(fr, FREQS, true_psd, left=0.0, right=0.0)
    z   = (rng.normal(size=len(fr)) + 1j * rng.normal(size=len(fr))) / np.sqrt(2.0)
    return np.fft.irfft(np.sqrt(psd * SAMPLE_RATE * n / 2.0) * z, n=n)


def welch_full(x: np.ndarray) -> np.ndarray:
    """PSD on the full rfft grid, exactly as WelchRunnable computes it."""
    _, p = welch(x, fs=SAMPLE_RATE, window='hann', nperseg=SPEC_N,
                 noverlap=SPEC_N // 2, scaling='density')
    return np.maximum(p, 1e-30)


def measure(true_psd: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """One 2 s control window's PSD estimate on FREQS."""
    return welch_full(noise(true_psd, rng))[MASK]


def on_target(rng: np.random.Generator, gain_db: float = 0.0,
              offset_db: float = 0.0, profile=PROFILE) -> np.ndarray:
    """A window measured from a rig sitting offset_db away from demand."""
    band = (FREQS >= profile[0][0]) & (FREQS <= profile[-1][0])
    true = np.where(band, psd_interp_loglog(FREQS, profile, gain_db + offset_db), 0.0)
    return measure(true, rng)


class SimRig:
    """measured = plant × drive, with the drive built the way the audio worker
    builds it: profile × correction, band-limited, normalised to unit power,
    then scaled by the output level.

    coupling > 0 adds a resonance at res_hz excited by the TOTAL drive power —
    a non-diagonal plant, where cutting the drive at the resonance does not
    bring the measured level there down.
    """

    def __init__(self, plant_db: np.ndarray, rng: np.random.Generator,
                 profile=PROFILE, coupling: float = 0.0,
                 res_hz: float = 600.0, res_width_hz: float = 15.0) -> None:
        self.plant    = 10.0 ** (np.asarray(plant_db, dtype=np.float64) / 10.0)
        self.rng      = rng
        self.profile  = profile
        self.band     = (FREQS >= profile[0][0]) & (FREQS <= profile[-1][0])
        self.coupling = coupling
        self.res      = np.exp(-0.5 * ((FREQS - res_hz) / res_width_hz) ** 2)

    def true_psd(self, corr_db: np.ndarray, out_dbfs: float) -> np.ndarray:
        shape = np.where(
            self.band,
            psd_interp_loglog(FREQS, self.profile) * 10.0 ** (corr_db / 10.0), 0.0)
        shape = shape / np.sum(shape)                       # unit power, df = 1 Hz
        level = 10.0 ** (out_dbfs / 10.0)
        true  = self.plant * level * shape
        if self.coupling:
            true = true + self.coupling * level * self.res  # fed by the whole band
        return true

    def measure(self, corr_db: np.ndarray, out_dbfs: float) -> np.ndarray:
        return measure(self.true_psd(corr_db, out_dbfs), self.rng)


# Where the simulated tests start the drive. Low enough that a rig 10–15 dB
# short of demand is still reached well below the controller's drive ceiling —
# these rigs have an amplifier with gain to spare. Tests about running OUT of
# drive build their rig against the ceiling instead.
START_DBFS = -30.0


def flat_plant_db(deficit_db: float, profile=PROFILE, at_dbfs: float = START_DBFS) -> float:
    """Plant gain that leaves the rig deficit_db below demand at at_dbfs."""
    band   = (FREQS >= profile[0][0]) & (FREQS <= profile[-1][0])
    demand = float(np.sum(psd_interp_loglog(FREQS, profile)[band]))
    return 10.0 * np.log10(demand) - at_dbfs - deficit_db


def controller() -> SpectralController:
    """A controller with its level seeded at START_DBFS."""
    ctl = SpectralController(FREQS)
    ctl.reseed_level(START_DBFS)
    return ctl
