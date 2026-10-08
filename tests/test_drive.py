"""Drive synthesis and the audio worker's block rendering (tickets 04, 09)."""

import numpy as np
import pytest

PROFILE = [(20.0, 0.0005), (2000.0, 0.0005)]


def rms_db(x):
    return 20 * np.log10(np.sqrt(np.mean(np.square(x, dtype=np.float64))))


def test_block_is_unit_rms_whatever_gain_the_profile_carries(iv, rng):
    """Why a sequence step must be applied as output level, not here."""
    for gain_db in (-12.0, -6.0, 0.0, 6.0):
        sig = iv._generate_shaped_block(PROFILE, gain_db, 4096, 44100, rng)
        assert np.sqrt(np.mean(sig.astype(np.float64) ** 2)) == pytest.approx(1.0, abs=1e-5)


def test_block_is_confined_to_the_profile_band(iv, rng):
    n, fs = 4096, 44100
    spec  = np.zeros(n // 2 + 1)
    for _ in range(50):
        spec += np.abs(np.fft.rfft(iv._generate_shaped_block(PROFILE, 0.0, n, fs, rng))) ** 2
    f = np.fft.rfftfreq(n, 1 / fs)
    inside = spec[(f >= 20) & (f <= 2000)].sum()
    assert inside / spec.sum() > 0.9999


def test_correction_shapes_the_block_and_tapers_outside_its_span(iv, rng):
    n, fs  = 4096, 44100
    cf     = np.array([100.0, 500.0, 501.0, 1000.0])
    cdb    = np.array([0.0, 0.0, 20.0, 20.0])        # +20 dB above 500 Hz, to 1 kHz only
    spec   = np.zeros(n // 2 + 1)
    for _ in range(200):
        spec += np.abs(np.fft.rfft(
            iv._generate_shaped_block(PROFILE, 0.0, n, fs, rng, cf, cdb))) ** 2
    f = np.fft.rfftfreq(n, 1 / fs)
    lo  = spec[(f > 150) & (f < 450)].mean()
    mid = spec[(f > 550) & (f < 950)].mean()
    hi  = spec[(f > 1100) & (f < 1900)].mean()       # past the correction's span
    assert 10 * np.log10(mid / lo) == pytest.approx(20.0, abs=1.0)
    assert 10 * np.log10(hi / lo) == pytest.approx(0.0, abs=1.0)


def test_output_gain_is_the_level_control(iv, rng):
    w = iv.AudioOutputWorker()
    w.set_profile(PROFILE, 0.0)
    levels = {}
    for db in (-26.0, -20.0):
        w.set_output_gain(10 ** (db / 20))
        levels[db] = rms_db(np.concatenate([w.render(4096, rng)[0] for _ in range(20)]))
    assert levels[-20.0] - levels[-26.0] == pytest.approx(6.0, abs=0.01)
    assert levels[-20.0] == pytest.approx(-20.0, abs=0.01)


def test_every_clipping_block_is_flagged(iv, rng):
    """Ticket 09: the flag used to be raised on the first clipping block only."""
    w = iv.AudioOutputWorker()
    w.set_profile(PROFILE, 0.0)
    w.set_output_gain(1.0)                           # unit-RMS noise at full scale clips
    flags = []
    for _ in range(200):
        sig, clipped = w.render(4096, rng)
        flags.append(clipped)
        assert np.max(np.abs(sig)) <= 1.0            # and it is hard-limited
    assert all(flags)

    w.set_output_gain(0.1)                           # -20 dB: 10 sigma of headroom
    assert not any(w.render(4096, rng)[1] for _ in range(200))
