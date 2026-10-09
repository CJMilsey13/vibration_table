"""Resampling the sensor's own sample rate to exactly 8000 Hz (ticket 14).

The sensor runs on its internal oscillator — 8107.4 Hz on the development
unit, not 8000. Read once per sample, the stream arrives at that rate, and
every frequency would otherwise be displayed 1.3 % low.
"""

import numpy as np
import pytest
from scipy.signal import welch

from stream import (
    CTRL_SAMPLES, INPUT_RATE_MAX, INPUT_RATE_MIN, SAMPLE_RATE, MeasurementStream,
    Resampler, banner_rate,
)

SENSOR_HZ = 8107.419        # as reported by the firmware on the development unit
BANNER    = ('# icm42688_streamer 2026-10-08 drdy-polled odr=8107.419 '
             'reset=11,0D,30,40,62 now=01,0D,7E,80,3F')


def tone(freq, seconds, rate=SENSOR_HZ, channels=1):
    t = np.arange(int(rate * seconds)) / rate
    return np.repeat(np.sin(2 * np.pi * freq * t)[:, None], channels, axis=1)


def peak_hz(y, rate=SAMPLE_RATE):
    """Frequency of the strongest line, interpolated to well under one bin."""
    w = np.hanning(len(y))
    Y = np.abs(np.fft.rfft(y * w))
    k = int(Y.argmax())
    a, b, c = np.log(Y[k - 1:k + 2])
    return (k + 0.5 * (a - c) / (a - 2 * b + c)) * rate / len(y)


def level_db(y):
    return 20 * np.log10(np.sqrt(2 * np.mean(y ** 2)))


# ── Resampler ────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('freq', [20.0, 100.0, 1000.0, 2000.0, 3000.0])
def test_tone_keeps_its_frequency_and_level(freq):
    y = Resampler(1, SENSOR_HZ, SAMPLE_RATE).process(tone(freq, 8.0))[1000:, 0]
    assert peak_hz(y) == pytest.approx(freq, abs=0.02)
    assert level_db(y) == pytest.approx(0.0, abs=0.01)


def test_without_resampling_the_same_tone_reads_low():
    """What the host showed before: 2000 Hz at 1973 Hz."""
    assert peak_hz(tone(2000.0, 8.0)[:, 0]) == pytest.approx(
        2000.0 * SAMPLE_RATE / SENSOR_HZ, abs=0.02)
    assert 2000.0 * SAMPLE_RATE / SENSOR_HZ == pytest.approx(1973.5, abs=0.1)


def test_output_rate_is_exact():
    r = Resampler(1, SENSOR_HZ, SAMPLE_RATE)
    seconds = 20.0
    n = sum(len(r.process(c)) for c in np.array_split(tone(100.0, seconds), 2000))
    assert n == pytest.approx(seconds * SAMPLE_RATE, abs=Resampler.HALF + 2)


def test_chunking_is_invisible(rng):
    x = rng.normal(size=(40000, 3))
    whole = Resampler(3, SENSOR_HZ, SAMPLE_RATE).process(x)
    r = Resampler(3, SENSOR_HZ, SAMPLE_RATE)
    pieces, i = [], 0
    for size in rng.integers(1, 400, size=1000):
        pieces.append(r.process(x[i:i + size]))
        i += size
        if i >= len(x):
            break
    chunked = np.concatenate(pieces)
    assert len(chunked) == len(whole)                    # no sample gained or lost at a seam
    # Same samples, to rounding: the output position is accumulated in a
    # different order of additions, which moves it by ~1e-11 of a sample.
    assert np.max(np.abs(chunked - whole)) < 1e-9


def test_noise_spectrum_is_untouched_in_band(rng):
    x = rng.normal(size=(int(SENSOR_HZ * 60), 1))
    y = Resampler(1, SENSOR_HZ, SAMPLE_RATE).process(x)
    fi, pi = welch(x[:, 0], fs=SENSOR_HZ, nperseg=8192)
    fo, po = welch(y[:, 0], fs=SAMPLE_RATE, nperseg=8192)
    for lo, hi in ((20, 500), (500, 1000), (1000, 2000), (2000, 3000)):
        di = np.mean(pi[(fi >= lo) & (fi < hi)])        # g²/Hz — rate-independent
        do = np.mean(po[(fo >= lo) & (fo < hi)])
        assert 10 * np.log10(do / di) == pytest.approx(0.0, abs=0.15), (lo, hi)


def test_nothing_above_the_output_nyquist_folds_back():
    # 4045 Hz exists at 8107 Hz sampling but not at 8000; unfiltered it would
    # alias to 3955 Hz at full level.
    y = Resampler(1, SENSOR_HZ, SAMPLE_RATE).process(tone(4045.0, 4.0))[1000:, 0]
    assert level_db(y) < -60.0


def test_dc_gain_is_exactly_one():
    y = Resampler(3, SENSOR_HZ, SAMPLE_RATE).process(np.ones((4000, 3)))
    assert np.allclose(y[200:], 1.0, atol=1e-12)        # 1 g stays 1 g


def test_rate_can_be_followed_without_a_glitch():
    r = Resampler(1, SENSOR_HZ, SAMPLE_RATE)
    x = tone(1000.0, 4.0)
    a = r.process(x[:16000])
    r.set_rate(SENSOR_HZ * (1 + 20e-6))                 # 20 ppm, as the firmware re-measures
    b = r.process(x[16000:])
    y = np.concatenate([a, b])[1000:, 0]
    assert np.max(np.abs(np.diff(y))) < 2 * np.pi * 1000 / SAMPLE_RATE * 1.01   # no step
    assert level_db(y) == pytest.approx(0.0, abs=0.01)


# ── Status line ──────────────────────────────────────────────────────────────

def test_banner_rate():
    assert banner_rate(BANNER) == 8107.419
    assert banner_rate('# icm42688_streamer x odr=0.000 reset=…') is None   # not measured yet
    assert banner_rate('# icm42688_streamer x odr=9000.000') is None        # not believable
    assert banner_rate('# icm42688_streamer 2026-10-08 drdy-polled') is None
    assert banner_rate('') is None
    assert INPUT_RATE_MIN < 8107.419 < INPUT_RATE_MAX


# ── MeasurementStream ────────────────────────────────────────────────────────

def test_stream_at_nominal_rate_is_a_pass_through(rng):
    s = MeasurementStream(3)
    x = rng.normal(size=(800, 3)).astype(np.float32)
    s.push(x)
    assert s.n_samples == 800 and np.array_equal(s.recent_raw(800), x)
    assert s.input_rate == SAMPLE_RATE


def test_first_rate_report_restarts_the_stream_later_ones_do_not():
    s = MeasurementStream(3)
    s.push(np.zeros((4000, 3), dtype=np.float32))
    epoch = s.epoch
    assert s.set_input_rate(SENSOR_HZ) is True          # data so far was on the wrong time base
    assert s.n_samples == 0 and s.epoch == epoch + 1 and s.input_rate == SENSOR_HZ
    s.push(np.zeros((4000, 3), dtype=np.float32))
    n = s.n_samples
    assert s.set_input_rate(SENSOR_HZ + 0.05) is False  # the firmware's next measurement
    assert s.n_samples == n and s.epoch == epoch + 1
    assert s.set_input_rate(SAMPLE_RATE) is True        # a device at the nominal rate again
    assert s.input_rate == SAMPLE_RATE


def test_unbelievable_rate_is_ignored():
    s = MeasurementStream(3)
    assert s.set_input_rate(0.0) is False and s.set_input_rate(9000.0) is False
    assert s.input_rate == SAMPLE_RATE


def test_window_from_a_sensor_rate_stream_is_on_the_true_frequency_axis():
    s = MeasurementStream(3)
    s.set_input_rate(SENSOR_HZ)
    x = tone(1737.0, 4.0, channels=3).astype(np.float32)     # an ambient line seen on the rig
    window = None
    for i in range(0, len(x), 80):
        s.push(x[i:i + 80])
        w = s.take_window()
        if w is not None:
            window = w
    assert window.samples.shape == (CTRL_SAMPLES, 3)
    f, p = welch(window.samples[:, 2], fs=SAMPLE_RATE, window='hann', nperseg=8000, noverlap=4000)
    assert f[p.argmax()] == 1737.0                           # not 1714
    # Seconds are seconds again: 4 s of sensor data is 4 s of stream.
    assert s.n_samples == pytest.approx(4.0 * SAMPLE_RATE, abs=Resampler.HALF + 2)


def test_reset_clears_the_resampler_too():
    s = MeasurementStream(1)
    s.set_input_rate(SENSOR_HZ)
    s.push(np.full((4000, 1), 5.0, dtype=np.float32))
    s.reset()
    s.push(np.zeros((400, 1), dtype=np.float32))
    assert np.all(s.recent_raw(s.n_samples) == 0.0)          # nothing left over from before
