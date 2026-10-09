"""
Measurement stream — ring buffers, bandpass state and the two Welch gates.

Qt-free. Owns every counter that decides *when* a PSD may be computed, so a
reconnect resets all of them together or none of them.

Samples come IN at the sensor's own rate and leave at exactly SAMPLE_RATE.
The sensor's output data rate is set by its internal oscillator — nominally
8 kHz, measured at 8108 Hz on the development unit — and the firmware delivers
every sensor sample exactly once, so the stream arrives at that rate. It is
resampled here, using the rate the firmware reports, before anything else sees
it. Everything downstream can then go on treating one sample as 1/8000 s.

Two rates leave this module and they must never be re-merged:

  display  one window every DISPLAY_WELCH_SAMPLES of new data; consecutive
           windows overlap heavily, which is fine for a live trace.
  control  a window is flagged `is_control` only once CTRL_SAMPLES of new data
           have arrived since the previous control window, so consecutive
           control windows share no samples.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

SAMPLE_RATE = 8000.0
WINDOW_TIME = 10.0                          # max ring-buffer / display length
HISTORY     = int(SAMPLE_RATE * WINDOW_TIME)  # 80 000 samples

SPEC_N            = 8000   # 1 s segments → 1 Hz resolution

# The control loop must see a FRESH, NON-OVERLAPPING measurement each update.
# Welch therefore runs over the most recent CTRL_SAMPLES only, and is launched
# only once that many new samples have arrived — so consecutive control updates
# share no data. Measuring over the whole ring while updating every batch made
# the loop react ~1000x faster than its own measurement could refresh, which
# wound the correction to the clamp and produced a sustained limit cycle.
CTRL_WINDOW_S     = 2.0
CTRL_SAMPLES      = int(SAMPLE_RATE * CTRL_WINDOW_S)   # 16000 → 3 Welch averages
WELCH_MIN_SAMPLES = CTRL_SAMPLES

# Welch itself runs fast so the spectrum feels live; consecutive windows overlap
# heavily, which is fine for display. The CONTROL update is gated separately on
# CTRL_SAMPLES of new data, so it still only ever sees non-overlapping windows.
DISPLAY_WELCH_HZ      = 15.0
DISPLAY_WELCH_SAMPLES = max(1, int(SAMPLE_RATE / DISPLAY_WELCH_HZ))

# After a drive change the loop did not command (a sequence step, Drive On, the
# slider, a reset), wait this long before starting the next control window, so
# the audio path's own latency is not measured as part of the new drive.
CTRL_SETTLE_S       = 0.25
CTRL_SETTLE_SAMPLES = int(SAMPLE_RATE * CTRL_SETTLE_S)

PSD_FMIN = 5.0
PSD_FMAX = 4000.0

# A reported sensor rate outside this range is not believed (±5 % of nominal;
# the part's oscillator tolerance is about ±1–2 %).
INPUT_RATE_MIN = 0.95 * SAMPLE_RATE
INPUT_RATE_MAX = 1.05 * SAMPLE_RATE
# The firmware re-measures the rate every few seconds and it wanders by a few
# ppm. Changes that small are followed smoothly. A larger one means the first
# report of a session has just arrived (or a different device has), so the
# data gathered under the old assumption is discarded.
INPUT_RATE_RESET_FRAC = 1e-3


def banner_rate(line: str) -> Optional[float]:
    """Sensor sample rate, in Hz, from the firmware's status line

        # icm42688_streamer 2026-10-08 drdy-polled odr=8107.419 reset=… now=…

    None if the line carries no usable rate — including odr=0.000, which the
    firmware sends until its first measurement is in."""
    m = re.search(r'\bodr=([0-9]+(?:\.[0-9]+)?)', line)
    if not m:
        return None
    rate = float(m.group(1))
    return rate if INPUT_RATE_MIN <= rate <= INPUT_RATE_MAX else None


class Resampler:
    """Streaming arbitrary-ratio resampler (windowed-sinc interpolation).

    Each output sample is a weighted sum of the 2·HALF input samples around
    its position in time. The kernel is a low-pass at 46 % of the LOWER of the
    two rates, so the response is flat to about 3.3 kHz and nothing above the
    output Nyquist is folded back in. Chunk boundaries are invisible: feeding
    a signal in any chunking gives the same output samples.
    """

    HALF   = 32      # kernel half-width, input samples (≈4 ms of delay)
    PHASES = 512     # kernel table resolution; linearly interpolated between
    BETA   = 9.0     # Kaiser window — about 90 dB of stop-band

    def __init__(self, channels: int, in_rate: float, out_rate: float) -> None:
        self._channels = channels
        self._out_rate = out_rate
        self._taps     = np.arange(-self.HALF + 1, self.HALF + 1)
        self.set_rate(in_rate)
        self.reset()

    def set_rate(self, in_rate: float) -> None:
        """Follow a small change in input rate. Keeps the filter state."""
        self._ratio = in_rate / self._out_rate          # input samples per output sample
        fc    = 0.46 * min(in_rate, self._out_rate) / in_rate      # cycles per input sample
        phase = np.arange(self.PHASES + 1) / self.PHASES
        d     = self._taps[None, :] - phase[:, None]    # tap position minus output position
        u     = np.clip(d / self.HALF, -1.0, 1.0)
        table = (2.0 * fc * np.sinc(2.0 * fc * d)
                 * np.i0(self.BETA * np.sqrt(1.0 - u * u)) / np.i0(self.BETA))
        self._table = table / table.sum(axis=1, keepdims=True)     # exactly unity at DC

    def reset(self) -> None:
        self._buf = np.zeros((2 * self.HALF, self._channels), dtype=np.float64)
        self._pos = float(self.HALF)                    # next output, in _buf coordinates

    def process(self, x: np.ndarray) -> np.ndarray:
        buf = np.concatenate([self._buf, np.asarray(x, dtype=np.float64)])
        # Outputs whose kernel is fully inside the data we hold.
        n = int(np.ceil((len(buf) - self.HALF - self._pos) / self._ratio))
        if n <= 0:
            self._buf = buf
            return np.empty((0, self._channels), dtype=np.float64)
        pos  = self._pos + self._ratio * np.arange(n)
        base = np.floor(pos).astype(np.int64)
        while n and base[n - 1] + self.HALF > len(buf) - 1:        # rounding at the edge
            n -= 1
        pos, base = pos[:n], base[:n]

        f = (pos - base) * self.PHASES
        j = np.minimum(f.astype(np.int64), self.PHASES - 1)
        a = (f - j)[:, None]
        w = (1.0 - a) * self._table[j] + a * self._table[j + 1]    # (n, taps)
        y = np.einsum('nk,nkc->nc', w, buf[base[:, None] + self._taps[None, :]])

        nxt   = self._pos + self._ratio * n
        start = int(np.floor(nxt)) - self.HALF + 1
        self._buf = buf[start:]
        self._pos = nxt - start
        return y


@dataclass(frozen=True)
class Window:
    """The most recent CTRL_SAMPLES of bandpassed data, oldest sample first."""
    samples:    np.ndarray   # (CTRL_SAMPLES, channels) float64
    is_control: bool         # True → shares no samples with the last control window
    epoch:      int          # stream session; a result from an old one is stale
    ctrl_gen:   int          # bumped by restart_control_window()


class MeasurementStream:

    def __init__(self, channels: int, fs: float = SAMPLE_RATE) -> None:
        self._channels = channels
        self._fs       = fs
        self._sos      = butter(
            4, [PSD_FMIN, min(PSD_FMAX - 5.0, fs * 0.49)],
            btype='bandpass', fs=fs, output='sos')
        self._ring      = np.zeros((HISTORY, channels), dtype=np.float32)
        self._ring_filt = np.zeros((HISTORY, channels), dtype=np.float32)
        self._disp      = np.zeros((HISTORY, channels), dtype=np.float32)
        self._epoch     = 0
        # None while samples arrive at exactly fs — then push() is a pass-through.
        self._input_rate = fs
        self._resampler: Optional[Resampler] = None
        self.reset()

    def reset(self) -> None:
        """Start a new session. Clears data, filter state and BOTH gates."""
        self._ring[:]      = 0.0
        self._ring_filt[:] = 0.0
        self._zi           = np.stack([sosfilt_zi(self._sos)] * self._channels, axis=-1)
        self._ptr          = 0
        self._n_samples    = 0
        self._last_welch_n = 0   # gates display Welch rate
        self._last_ctrl_n  = 0   # gates control updates (fresh windows)
        self._ctrl_gen     = 0
        self._epoch       += 1
        if self._resampler is not None:
            self._resampler.reset()

    @property
    def input_rate(self) -> float:
        """The rate samples are arriving at, as last reported."""
        return self._input_rate

    def set_input_rate(self, rate: float) -> bool:
        """Say what rate push() is being fed at. True if that reset the stream.

        A rate that is out of range is ignored. A change of more than
        INPUT_RATE_RESET_FRAC — the first report of a session — discards what
        has been gathered so far, because it was laid down on the wrong time
        base; reset() is called and the caller should drop anything it
        averaged from this stream. Smaller changes are just followed.
        """
        if not INPUT_RATE_MIN <= rate <= INPUT_RATE_MAX:
            return False
        nominal_now  = self._resampler is None
        nominal_next = rate == self._fs
        step = abs(rate / self._input_rate - 1.0)
        self._input_rate = rate
        if nominal_now != nominal_next or step > INPUT_RATE_RESET_FRAC:
            self._resampler = None if nominal_next else Resampler(
                self._channels, rate, self._fs)
            self.reset()
            return True
        if self._resampler is not None:
            self._resampler.set_rate(rate)
        return False

    @property
    def n_samples(self) -> int:
        return self._n_samples

    @property
    def epoch(self) -> int:
        return self._epoch

    def push(self, batch: np.ndarray) -> None:
        if self._resampler is not None:
            batch = self._resampler.process(batch)
            if not len(batch):
                return
        n = len(batch)
        filt, self._zi = sosfilt(self._sos, batch, axis=0, zi=self._zi)

        p = self._ptr
        if p + n <= HISTORY:
            self._ring[p:p+n]      = batch
            self._ring_filt[p:p+n] = filt
        else:
            first = HISTORY - p
            self._ring[p:]      = batch[:first]; self._ring[:n-first]      = batch[first:]
            self._ring_filt[p:] = filt[:first];  self._ring_filt[:n-first] = filt[first:]

        self._ptr        = (p + n) % HISTORY
        self._n_samples += n

    def take_window(self) -> Optional[Window]:
        """Return a window if one is due, else None. Taking it advances the gates."""
        n_now = self._n_samples
        if (n_now < WELCH_MIN_SAMPLES
                or n_now - self._last_welch_n < DISPLAY_WELCH_SAMPLES):
            return None
        self._last_welch_n = n_now

        # Decided on the window's own end sample, so two control windows can
        # never overlap however long the PSD takes to compute.
        is_control = (n_now - self._last_ctrl_n) >= CTRL_SAMPLES
        if is_control:
            self._last_ctrl_n = n_now

        n = CTRL_SAMPLES
        # Most recent n samples, ending at the write pointer.
        start = (self._ptr - n) % HISTORY
        if start + n <= HISTORY:
            snap = self._ring_filt[start:start + n].astype(np.float64)
        else:
            first = HISTORY - start
            snap = np.empty((n, self._channels), dtype=np.float64)
            snap[:first] = self._ring_filt[start:]
            snap[first:] = self._ring_filt[:n - first]
        return Window(snap, is_control, self._epoch, self._ctrl_gen)

    def restart_control_window(self) -> None:
        """The drive just changed in a way the loop did not command.

        Everything already in the ring was measured under the OLD drive. A
        control update fed that data would see the change as an error and
        "correct" it a second time — a sequence step applied twice, or ambient
        noise read as a 40 dB shortfall the instant the drive comes on. So the
        next control window is gathered entirely after this call, and a window
        already handed out no longer counts as one (see is_control_window).
        """
        self._last_ctrl_n = self._n_samples + CTRL_SETTLE_SAMPLES
        self._ctrl_gen   += 1

    def is_current(self, window: Window) -> bool:
        """False for a window taken before the last reset()."""
        return window.epoch == self._epoch

    def is_control_window(self, window: Window) -> bool:
        """May this window drive a control update NOW? Re-checked when its PSD
        arrives, because the drive can change while the PSD is being computed."""
        return (window.is_control and window.epoch == self._epoch
                and window.ctrl_gen == self._ctrl_gen)

    def recent_raw(self, n: int) -> np.ndarray:
        """Last n unfiltered samples, oldest first, for the time-domain plot.

        The returned array is a view of an internal buffer — valid until the
        next call.
        """
        p = self._ptr
        self._disp[:HISTORY - p] = self._ring[p:]
        self._disp[HISTORY - p:] = self._ring[:p]
        return self._disp[-n:]
