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


@pytest.mark.parametrize('fs', [44100, 48000, 96000])
def test_block_is_confined_to_the_profile_band(iv, rng, fs):
    n     = 4096
    half  = 0.5 * fs / n                             # half a drive bin
    spec  = np.zeros(n // 2 + 1)
    for _ in range(50):
        spec += np.abs(np.fft.rfft(iv._generate_shaped_block(PROFILE, 0.0, n, fs, rng))) ** 2
    f = np.fft.rfftfreq(n, 1 / fs)
    inside = spec[(f >= 20 - half) & (f <= 2000 + half)].sum()
    assert inside / spec.sum() > 0.9999              # nothing beyond half a bin of slack
    assert spec[f > 2000 + half].sum() / spec.sum() < 1e-6    # and nothing toward Nyquist


@pytest.mark.parametrize('fs', [44100, 48000])
def test_drive_reaches_the_edges_of_the_band(iv, rng, fs):
    """The top few Hz of the band used to get no drive: the last drive bin
    centred inside 20–2000 Hz is at 1992 Hz and covers only to ~1998 Hz. The
    loop then wound those bins toward the boost rail."""
    x = np.concatenate([iv._generate_shaped_block(PROFILE, 0.0, 4096, fs, rng)
                        for _ in range(600)]).astype(np.float64)
    from scipy.signal import welch
    f, p = welch(x, fs=fs, nperseg=fs, noverlap=fs // 2)        # 1 Hz bins, like the host
    mid  = p[(f >= 1800) & (f <= 1950)].mean()
    for lo, hi in ((1996, 2000), (20, 24)):
        edge = p[(f >= lo) & (f <= hi)].mean()
        assert 10 * np.log10(edge / mid) > -1.5, (lo, hi)       # was -4.2 dB at the top


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


def test_clip_flag_means_distortion_not_one_stray_peak(iv, rng):
    w = iv.AudioOutputWorker()
    w.set_profile(PROFILE, 0.0)
    # At the default ceiling the odd 4-sigma peak is flattened — about one
    # sample in 15 000, 50 dB down. That is not what the indicator is for.
    w.set_output_gain(10 ** (-12 / 20))
    blocks = [w.render(4096, rng) for _ in range(400)]
    assert not any(clipped for _, clipped in blocks)
    assert max(float(np.max(np.abs(sig))) for sig, _ in blocks) <= 1.0     # still hard-limited
    # 3 dB hotter, 0.5 % of samples clip (distortion ~30 dB down): flagged in
    # enough blocks to hold the indicator on, which needs one every 2 s.
    w.set_output_gain(10 ** (-9 / 20))
    assert sum(w.render(4096, rng)[1] for _ in range(400)) > 80
    # By -6 dBFS (4.6 % clipped, 20 dB down) it is every block.
    w.set_output_gain(10 ** (-6 / 20))
    assert all(w.render(4096, rng)[1] for _ in range(100))
