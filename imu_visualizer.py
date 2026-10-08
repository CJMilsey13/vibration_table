"""
IMU Visualizer — Accel X/Y/Z time-series + Welch PSD + Random Vibration Test Profile
======================================================================================
Usage:
  python imu_visualizer.py              # connect to real hardware
  python imu_visualizer.py --demo       # run with simulated data (no hardware)

Requirements:  pip install pyqtgraph PyQt5 pyserial numpy scipy
Hardware:      ICM-42688-P via SPI → Pimoroni Pico Plus 2 (RP2350) → USB CDC

Wire protocol (firmware → host):
  [0xAA][0x55] | seq uint16 LE | ax int16 LE | ay int16 LE | az int16 LE
  = 10 bytes per sample at 8 kHz nominal
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from pathlib import Path
from typing import Optional

import threading

import numpy as np
import pyqtgraph as pg
from scipy.signal import butter, sosfilt, sosfilt_zi, welch as scipy_welch
from scipy.ndimage import gaussian_filter1d
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtGui, QtWidgets

try:
    import sounddevice as _sd
    HAS_SOUNDDEVICE = True
except ImportError:
    _sd = None          # type: ignore[assignment]
    HAS_SOUNDDEVICE = False


# ── Constants ─────────────────────────────────────────────────────────────────

CHANNEL_DICT: dict[str, tuple[int, int, int]] = {
    'Accel X': (255, 200,   0),
    'Accel Y': (210,  80, 255),
    'Accel Z': (  0, 215, 215),
}
CHANNEL_COUNT = len(CHANNEL_DICT)
CHANNEL_NAMES = list(CHANNEL_DICT.keys())

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
PSD_DISPLAY_TAU_S     = 0.35   # EMA time constant, display only — never control

CTRL_DEADBAND_DB  = 0.5    # ≈1σ of the smoothed estimate; soft-thresholded
CTRL_MAX_STEP_DB  = 6.0    # per-update slew limit on the correction

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
BATCH    = 80
PSD_FMIN = 5.0
PSD_FMAX = 4000.0
PSD_FLOOR_LOG = -12.0   # log10(g²/Hz) placeholder for "no data yet"
# Spectrum view window, in decades either side of the demand profile. Kept tight
# on purpose: the ±6 dB tolerance band is only 0.6 of a decade, so a very wide
# view compresses the one thing the operator is reading into a few pixels.
PSD_VIEW_DECADES_BELOW = -4.0
PSD_VIEW_DECADES_ABOVE =  1.0
PSD_VIEW_MAX_DECADES   =  8.0   # hard cap, so a clamped bin can't blow the scale
# Tolerance bands drawn around the demand profile (dB power), the usual way a
# random-vibration run is judged in or out of spec.
TOL_ALARM_DB = 3.0
TOL_ABORT_DB = 6.0

SYNC_A, SYNC_B = 0xAA, 0x55
PAYLOAD_BYTES  = 8

FSR_OPTIONS: dict[str, float] = {
    '±16 g': 2048.0,
    '±8 g':  4096.0,
    '±4 g':  8192.0,
    '±2 g': 16384.0,
}
FSR_DEFAULT = '±16 g'

# ── Default test profile ──────────────────────────────────────────────────────

DEFAULT_BREAKPOINTS: list[tuple[float, float]] = [
    (  20.0, 0.0005),
    (2000.0, 0.0005),
]

DEFAULT_SEQUENCE: list[tuple[float, float]] = [   # (duration_s, gain_db)
    (300.0, -6.0),
    (300.0,  0.0),
]

# Learned speaker/shaker response, reloaded at startup so the loop begins from
# the rig's known inverse transfer function instead of flat.
RESPONSE_FILE = Path(__file__).with_name('speaker_response.json')


# ── PSD profile helpers ───────────────────────────────────────────────────────

def psd_interp_loglog(
    freqs: np.ndarray,
    breakpoints: list[tuple[float, float]],
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


def psd_grms(breakpoints: list[tuple[float, float]], gain_db: float = 0.0) -> float:
    """Integrate log-log PSD profile analytically → Grms."""
    if len(breakpoints) < 2:
        return 0.0
    gain    = 10.0 ** (gain_db / 10.0)
    grms_sq = 0.0
    for i in range(len(breakpoints) - 1):
        f1, p1 = breakpoints[i][0],   breakpoints[i][1]   * gain
        f2, p2 = breakpoints[i+1][0], breakpoints[i+1][1] * gain
        m = math.log10(p2 / p1) / math.log10(f2 / f1)
        if abs(m + 1.0) < 1e-9:
            grms_sq += p1 * f1 * math.log(f2 / f1)
        else:
            grms_sq += p1 / (m + 1.0) * (f2**(m+1.0) - f1**(m+1.0)) / (f1**m)
    return math.sqrt(max(grms_sq, 0.0))


# ── Custom log-scale axis ─────────────────────────────────────────────────────

class LogHzAxis(pg.AxisItem):
    """Tick labels show plain Hz values instead of 10^x notation."""
    def tickStrings(self, values, scale, spacing):  # noqa: ARG002
        return [f'{10**v:.4g}' for v in values]


class LogPSDAxis(pg.AxisItem):
    """Tick labels show PSD in g²/Hz. Data is pre-log10'd, never setLogMode."""
    def tickStrings(self, values, scale, spacing):  # noqa: ARG002
        out = []
        for v in values:
            p = 10.0 ** v
            if p >= 0.1:
                out.append(f'{p:.3g}')
            else:
                out.append(f'{p:.0e}'.replace('e-0', 'e-'))
        return out


# ── Serial worker ─────────────────────────────────────────────────────────────

class SerialWorker(QtCore.QThread):
    batch_ready = QtCore.pyqtSignal(object, float, float, int)

    def __init__(self, port: str, baud: int, sensitivity: float) -> None:
        super().__init__()
        self.port        = port
        self.baud        = baud
        self.sensitivity = sensitivity
        self._running    = False

    def run(self) -> None:
        import time as _time

        FRAME_BYTES = 2 + PAYLOAD_BYTES
        SYNC        = bytes([SYNC_A, SYNC_B])
        READ_CHUNK  = 4096

        buf      = np.empty((BATCH, CHANNEL_COUNT), dtype=np.float32)
        idx      = 0
        t0       = 0.0
        drops    = 0
        last_seq: Optional[int] = None
        scale    = 1.0 / self.sensitivity
        pending  = bytearray()

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            ser.set_buffer_size(rx_size=131072)
            self._running = True

            while self._running:
                incoming = ser.read(READ_CHUNK)
                if incoming:
                    pending.extend(incoming)

                while len(pending) >= FRAME_BYTES:
                    sync_pos = pending.find(SYNC)
                    if sync_pos < 0:
                        pending = pending[-1:]
                        break
                    if sync_pos > 0:
                        pending = pending[sync_pos:]
                    if len(pending) < FRAME_BYTES:
                        break

                    seq, ax, ay, az = struct.unpack('<H3h', pending[2:FRAME_BYTES])
                    pending = pending[FRAME_BYTES:]

                    if last_seq is not None:
                        expected = (last_seq + 1) & 0xFFFF
                        if seq != expected:
                            drops += (seq - expected) & 0xFFFF
                    last_seq = seq

                    t = _time.monotonic()
                    if idx == 0:
                        t0 = t
                    buf[idx, 0] = ax * scale
                    buf[idx, 1] = ay * scale
                    buf[idx, 2] = az * scale
                    idx += 1

                    if idx == BATCH:
                        self.batch_ready.emit(buf.copy(), t0, t, drops)
                        drops = 0
                        idx   = 0

            ser.close()
        except serial.SerialException as exc:
            print(f'Serial error: {exc}')

    def stop(self) -> None:
        self._running = False
        self.wait(2000)


# ── Demo worker ───────────────────────────────────────────────────────────────

class DemoWorker(QtCore.QThread):
    """Synthetic 8 kHz data: 50 Hz + 120 Hz + 1 kHz peaks."""
    batch_ready = QtCore.pyqtSignal(object, float, float, int)

    def run(self) -> None:
        import time as _time
        self._running = True
        t   = 0.0
        dt  = 1.0 / SAMPLE_RATE
        buf = np.empty((BATCH, CHANNEL_COUNT), dtype=np.float32)
        idx = 0
        t0  = _time.monotonic()

        while self._running:
            buf[idx, 0] = (0.5 * math.sin(2*math.pi*50*t)
                         + 0.1 * math.sin(2*math.pi*1000*t))
            buf[idx, 1] =  0.3 * math.sin(2*math.pi*120*t + 1.0)
            buf[idx, 2] =  0.2 * math.sin(2*math.pi*75*t)
            t   += dt
            idx += 1
            if idx == BATCH:
                t1 = _time.monotonic()
                self.batch_ready.emit(buf.copy(), t0, t1, 0)
                idx = 0
                t0  = t1
                self.msleep(int(1000 * BATCH / SAMPLE_RATE))

    def stop(self) -> None:
        self._running = False
        self.wait(2000)


# ── Welch PSD runnable ────────────────────────────────────────────────────────

class WelchRunnable(QtCore.QRunnable):
    """50%-overlapping Hann-windowed Welch PSD, run in thread pool."""

    def __init__(self, snapshot: np.ndarray, fs: float, callback) -> None:
        super().__init__()
        self.setAutoDelete(True)
        self._snap     = snapshot
        self._fs       = fs
        self._callback = callback

    def run(self) -> None:
        results: list[np.ndarray] = []
        for ch in range(CHANNEL_COUNT):
            _, psd = scipy_welch(
                self._snap[:, ch],
                fs=self._fs,
                window='hann',
                nperseg=SPEC_N,
                noverlap=SPEC_N // 2,
                scaling='density',
            )
            results.append(np.maximum(psd, 1e-30))

        QtCore.QMetaObject.invokeMethod(
            self._callback.__self__,
            self._callback.__func__.__name__,
            QtCore.Qt.ConnectionType.QueuedConnection,
            QtCore.Q_ARG(object, results),
        )


# ── Audio output worker ───────────────────────────────────────────────────────

class AudioOutputWorker(QtCore.QThread):
    """
    Streams spectrally-shaped Gaussian noise to the system audio DAC.

    The PSD profile is IFFT-shaped each block: for each frequency bin f,
    amplitude = sqrt(PSD(f) * df), random phase.  The block is normalised to
    unit RMS before output_gain is applied, so output_gain alone controls the
    DAC level independent of the profile shape or absolute Grms.

    Clip detection fires clip_detected if any sample exceeds ±1.0 after
    output_gain scaling.
    """

    clip_detected = QtCore.pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self._breakpoints: list[tuple[float, float]] = list(DEFAULT_BREAKPOINTS)
        self._gain_db:     float = 0.0
        self._output_gain: float = 0.10   # linear; -20 dB default — safe start
        self._fs:          int   = 44100
        self._device:      int   = -1     # -1 = default output device
        self._block_size:  int   = 4096   # ≈ 93 ms per block at 44100 Hz
        self._running:     bool  = False
        self._corr_freqs:  Optional[np.ndarray] = None   # Hz, sorted ascending
        self._corr_db:     Optional[np.ndarray] = None   # dB power correction
        self._lock = threading.Lock()

    # called from Qt thread — protected by lock
    def set_profile(self, breakpoints: list[tuple[float, float]], gain_db: float) -> None:
        with self._lock:
            self._breakpoints = list(breakpoints)
            self._gain_db     = gain_db

    def set_output_gain(self, linear: float) -> None:
        with self._lock:
            self._output_gain = max(0.0, min(1.0, linear))

    def set_device(self, device_index: int) -> None:
        self._device = device_index   # safe only before run()

    def set_fs(self, fs: int) -> None:
        self._fs = fs                 # safe only before run()

    def set_correction(self, freqs: np.ndarray, corr_db: np.ndarray) -> None:
        """Update spectral correction from the control loop (Qt thread → audio thread)."""
        with self._lock:
            self._corr_freqs = freqs.copy()
            self._corr_db    = corr_db.copy()

    def clear_correction(self) -> None:
        with self._lock:
            self._corr_freqs = None
            self._corr_db    = None

    def run(self) -> None:
        if not HAS_SOUNDDEVICE:
            return

        rng = np.random.default_rng()   # per-thread RNG, never shared
        clipped_last = False

        def callback(outdata: np.ndarray, frames: int, _time, _status) -> None:
            nonlocal clipped_last
            with self._lock:
                bp      = self._breakpoints
                gain    = self._gain_db
                og      = self._output_gain
                corr_f  = self._corr_freqs
                corr_db = self._corr_db

            sig = _generate_shaped_block(bp, gain, frames, self._fs, rng, corr_f, corr_db)
            sig *= og

            if np.max(np.abs(sig)) > 1.0:
                np.clip(sig, -1.0, 1.0, out=sig)
                if not clipped_last:
                    clipped_last = True
                    self.clip_detected.emit()
            else:
                clipped_last = False

            outdata[:, 0] = sig
            if outdata.shape[1] > 1:
                outdata[:, 1] = sig

        kw = dict(
            samplerate=self._fs,
            channels=2,
            dtype='float32',
            blocksize=self._block_size,
            callback=callback,
        )
        if self._device >= 0:
            kw['device'] = self._device

        self._running = True
        try:
            with _sd.OutputStream(**kw):
                while self._running:
                    self.msleep(50)
        except Exception as exc:
            print(f'Audio output error: {exc}')

    def stop(self) -> None:
        self._running = False
        self.wait(2000)


def _generate_shaped_block(
    breakpoints: list[tuple[float, float]],
    gain_db: float,
    n: int,
    fs: int,
    rng: np.random.Generator,
    corr_freqs: Optional[np.ndarray] = None,
    corr_db: Optional[np.ndarray] = None,
) -> np.ndarray:
    """IFFT spectral-shaping: white noise coloured to match the PSD profile.

    corr_freqs / corr_db: optional per-bin correction from the control loop
    (dB power, same sign convention as error: positive = drive more).
    """
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    amp   = np.zeros(len(freqs), dtype=np.float64)
    if len(breakpoints) >= 2:
        # Synthesise ONLY inside the profile band. psd_interp_loglog clamps to the
        # edge breakpoint outside it, which would otherwise emit full-level noise
        # all the way to Nyquist. That energy cannot be measured or controlled, and
        # because the block is normalised to unit RMS below it directly steals level
        # from the in-band signal.
        f_lo = max(PSD_FMIN, breakpoints[0][0])
        f_hi = min(PSD_FMAX, breakpoints[-1][0], fs * 0.5)
        mask = (freqs >= f_lo) & (freqs <= f_hi)
    else:
        mask = np.zeros(len(freqs), dtype=bool)
    if mask.any():
        df       = fs / n
        psd_vals = psd_interp_loglog(freqs[mask], breakpoints, gain_db)
        if corr_freqs is not None and corr_db is not None and len(corr_freqs) >= 2:
            # Interpolate correction onto this block's grid (log-linear).
            # Taper to 0 dB outside the control band — holding a saturated edge
            # value here multiplies out-of-band power by up to 10^(max_corr/10).
            c = np.interp(
                np.log10(freqs[mask]),
                np.log10(corr_freqs),
                corr_db,
                left=0.0,
                right=0.0,
            )
            psd_vals = psd_vals * (10.0 ** (c / 10.0))
        amp[mask] = np.sqrt(np.maximum(psd_vals * df, 0.0))
    phases   = rng.uniform(0.0, 2.0 * np.pi, len(freqs))
    spectrum = amp * (np.cos(phases) + 1j * np.sin(phases))
    spectrum[0] = 0.0   # no DC
    sig = np.fft.irfft(spectrum, n=n).astype(np.float32)
    rms = float(np.sqrt(np.mean(sig ** 2)))
    if rms > 1e-12:
        sig /= rms      # normalise to unit RMS; output_gain is the only level control
    return sig


# ── Breakpoint table model ────────────────────────────────────────────────────

class BreakpointModel(QtCore.QAbstractTableModel):
    HEADERS = ['Freq (Hz)', 'Level (g²/Hz)']

    def __init__(self, rows: list | None = None, parent=None) -> None:
        super().__init__(parent)
        self._rows: list[list[float]] = [list(r) for r in (rows or DEFAULT_BREAKPOINTS)]

    def rowCount(self, parent=QtCore.QModelIndex()) -> int:   # noqa: ARG002
        return len(self._rows)

    def columnCount(self, parent=QtCore.QModelIndex()) -> int:  # noqa: ARG002
        return 2

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if role == QtCore.Qt.DisplayRole and orientation == QtCore.Qt.Horizontal:
            return self.HEADERS[section]
        return None

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid():
            return None
        v = self._rows[index.row()][index.column()]
        if role == QtCore.Qt.DisplayRole:
            return f'{v:.4g}'
        if role == QtCore.Qt.EditRole:
            return str(v)
        return None

    def setData(self, index, value, role=QtCore.Qt.EditRole) -> bool:
        if role != QtCore.Qt.EditRole:
            return False
        try:
            v = float(value)
        except (ValueError, TypeError):
            return False
        if v <= 0:
            return False
        self._rows[index.row()][index.column()] = v
        self._rows.sort(key=lambda r: r[0])
        self.layoutChanged.emit()
        return True

    def flags(self, index):
        return super().flags(index) | QtCore.Qt.ItemIsEditable

    def add_row(self) -> None:
        last = self._rows[-1] if self._rows else [1.0, 0.01]
        self._rows.append([last[0] * 2.0, last[1]])
        self._rows.sort(key=lambda r: r[0])
        self.layoutChanged.emit()

    def remove_row(self, row: int) -> None:
        if len(self._rows) <= 2:
            return
        self._rows.pop(row)
        self.layoutChanged.emit()

    def breakpoints(self) -> list[tuple[float, float]]:
        return [(r[0], r[1]) for r in self._rows]


# ── Sequence table model ──────────────────────────────────────────────────────

class SequenceModel(QtCore.QAbstractTableModel):
    HEADERS = ['Step', 'Duration (s)', 'Gain (dB)']

    def __init__(self, rows: list | None = None, parent=None) -> None:
        super().__init__(parent)
        self._rows: list[list[float]] = [list(r) for r in (rows or DEFAULT_SEQUENCE)]

    def rowCount(self, parent=QtCore.QModelIndex()) -> int:   # noqa: ARG002
        return len(self._rows)

    def columnCount(self, parent=QtCore.QModelIndex()) -> int:  # noqa: ARG002
        return 3

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if role == QtCore.Qt.DisplayRole and orientation == QtCore.Qt.Horizontal:
            return self.HEADERS[section]
        return None

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid():
            return None
        col, row = index.column(), index.row()
        if role == QtCore.Qt.DisplayRole:
            if col == 0:
                return str(row + 1)
            if col == 1:
                return f'{self._rows[row][0]:.4g}'
            return f'{self._rows[row][1]:+.1f}'
        if role == QtCore.Qt.EditRole:
            if col == 1:
                return str(self._rows[row][0])
            if col == 2:
                return str(self._rows[row][1])
        return None

    def setData(self, index, value, role=QtCore.Qt.EditRole) -> bool:
        if role != QtCore.Qt.EditRole:
            return False
        col = index.column()
        if col == 0:
            return False
        try:
            v = float(value)
        except (ValueError, TypeError):
            return False
        row = index.row()
        if col == 1:
            if v <= 0:
                return False
            self._rows[row][0] = v
        else:
            self._rows[row][1] = v
        self.dataChanged.emit(index, index)
        return True

    def flags(self, index):
        base = super().flags(index)
        return base if index.column() == 0 else base | QtCore.Qt.ItemIsEditable

    def add_row(self) -> None:
        self._rows.append([60.0, 0.0])
        self.layoutChanged.emit()

    def remove_row(self, row: int) -> None:
        if not self._rows:
            return
        self._rows.pop(row)
        self.layoutChanged.emit()

    def steps(self) -> list[tuple[float, float]]:
        return [(r[0], r[1]) for r in self._rows]

    def total_duration(self) -> float:
        return sum(r[0] for r in self._rows)


# ── Test Profile Dock ─────────────────────────────────────────────────────────

class TestProfileDock(QtWidgets.QDockWidget):
    profile_changed = QtCore.pyqtSignal()
    step_started    = QtCore.pyqtSignal(int, float)   # step_idx, gain_db
    test_started    = QtCore.pyqtSignal()              # fired once on Start
    test_stopped    = QtCore.pyqtSignal()
    save_response_requested = QtCore.pyqtSignal()
    load_response_requested = QtCore.pyqtSignal()
    audio_started           = QtCore.pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__('Test Profile', parent)
        self.setFeatures(
            QtWidgets.QDockWidget.DockWidgetMovable |
            QtWidgets.QDockWidget.DockWidgetFloatable
        )
        self._bp_model  = BreakpointModel()
        self._seq_model = SequenceModel()
        self._running   = False
        self._paused    = False
        self._step_idx  = 0
        self._elapsed_s = 0.0
        self._audio_worker: Optional[AudioOutputWorker] = None

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._tick)

        self._build_ui()
        self._bp_model.layoutChanged.connect(self._on_profile_changed)
        self._bp_model.dataChanged.connect(self._on_profile_changed)
        self._seq_model.layoutChanged.connect(self._update_total_label)
        self._seq_model.dataChanged.connect(self._update_total_label)

    def _build_ui(self) -> None:
        w = QtWidgets.QWidget()
        self.setWidget(w)
        vbox = QtWidgets.QVBoxLayout(w)
        vbox.setSpacing(6)
        vbox.setContentsMargins(6, 6, 6, 6)

        # ── PSD Breakpoints ───────────────────────────────────────────────
        bp_grp = QtWidgets.QGroupBox('PSD Breakpoints')
        bp_lay = QtWidgets.QVBoxLayout(bp_grp)

        self._bp_view = QtWidgets.QTableView()
        self._bp_view.setModel(self._bp_model)
        self._bp_view.horizontalHeader().setStretchLastSection(True)
        self._bp_view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self._bp_view.setMinimumHeight(120)
        self._bp_view.setMaximumHeight(190)
        bp_lay.addWidget(self._bp_view)

        bp_btns = QtWidgets.QHBoxLayout()
        for label, slot in (('+ Row', self._add_bp_row), ('− Row', self._del_bp_row)):
            btn = QtWidgets.QPushButton(label)
            btn.setFixedWidth(58)
            btn.clicked.connect(slot)
            bp_btns.addWidget(btn)
        bp_btns.addStretch()
        self._grms_base_lbl = QtWidgets.QLabel('Base Grms: —')
        self._grms_base_lbl.setStyleSheet('color: #aaa; font-family: Consolas;')
        bp_btns.addWidget(self._grms_base_lbl)
        bp_lay.addLayout(bp_btns)
        vbox.addWidget(bp_grp)

        # ── Control Channels ──────────────────────────────────────────────
        ch_grp = QtWidgets.QGroupBox('Control Channel(s)')
        ch_lay = QtWidgets.QHBoxLayout(ch_grp)
        self._ch_checks: dict[str, QtWidgets.QCheckBox] = {}
        for name, color in CHANNEL_DICT.items():
            cb = QtWidgets.QCheckBox(name)
            cb.setChecked(name == 'Accel Z')
            cb.setStyleSheet(f'color: rgb{color};')
            self._ch_checks[name] = cb
            ch_lay.addWidget(cb)
        ch_lay.addStretch()
        vbox.addWidget(ch_grp)

        # ── Test Sequence ─────────────────────────────────────────────────
        seq_grp = QtWidgets.QGroupBox('Test Sequence')
        seq_lay = QtWidgets.QVBoxLayout(seq_grp)

        self._seq_view = QtWidgets.QTableView()
        self._seq_view.setModel(self._seq_model)
        self._seq_view.horizontalHeader().setStretchLastSection(True)
        self._seq_view.setColumnWidth(0, 42)
        self._seq_view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self._seq_view.setMinimumHeight(100)
        self._seq_view.setMaximumHeight(170)
        seq_lay.addWidget(self._seq_view)

        seq_btns = QtWidgets.QHBoxLayout()
        for label, slot in (('+ Row', self._add_seq_row), ('− Row', self._del_seq_row)):
            btn = QtWidgets.QPushButton(label)
            btn.setFixedWidth(58)
            btn.clicked.connect(slot)
            seq_btns.addWidget(btn)
        seq_btns.addStretch()
        self._total_lbl = QtWidgets.QLabel('Total: —')
        self._total_lbl.setStyleSheet('color: #aaa; font-family: Consolas;')
        seq_btns.addWidget(self._total_lbl)
        seq_lay.addLayout(seq_btns)
        vbox.addWidget(seq_grp)

        # ── Test Control ──────────────────────────────────────────────────
        ctrl_grp = QtWidgets.QGroupBox('Test Control')
        ctrl_lay = QtWidgets.QVBoxLayout(ctrl_grp)

        run_row = QtWidgets.QHBoxLayout()
        self._start_btn = QtWidgets.QPushButton('▶  Start')
        self._pause_btn = QtWidgets.QPushButton('⏸  Pause')
        self._stop_btn  = QtWidgets.QPushButton('■  Stop')
        self._pause_btn.setEnabled(False)
        self._stop_btn.setEnabled(False)
        self._start_btn.clicked.connect(self._start_test)
        self._pause_btn.clicked.connect(self._pause_test)
        self._stop_btn.clicked.connect(self._stop_test)
        for btn in (self._start_btn, self._pause_btn, self._stop_btn):
            run_row.addWidget(btn)
        ctrl_lay.addLayout(run_row)

        self._step_lbl = QtWidgets.QLabel('Idle')
        self._step_lbl.setStyleSheet('color: #888; font-family: Consolas;')
        ctrl_lay.addWidget(self._step_lbl)

        self._progress = QtWidgets.QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        ctrl_lay.addWidget(self._progress)

        grms_row = QtWidgets.QHBoxLayout()
        self._grms_target_lbl = QtWidgets.QLabel('Target: — g')
        self._grms_meas_lbl   = QtWidgets.QLabel('Meas: — g')
        for lbl in (self._grms_target_lbl, self._grms_meas_lbl):
            lbl.setStyleSheet('color: #aaa; font-family: Consolas;')
        grms_row.addWidget(self._grms_target_lbl)
        grms_row.addStretch()
        grms_row.addWidget(self._grms_meas_lbl)
        ctrl_lay.addLayout(grms_row)

        vbox.addWidget(ctrl_grp)

        # ── Control Loop ──────────────────────────────────────────────────
        cl_grp = QtWidgets.QGroupBox('Control Loop')
        cl_lay = QtWidgets.QVBoxLayout(cl_grp)

        en_row = QtWidgets.QHBoxLayout()
        self._ctrl_enable_cb = QtWidgets.QCheckBox('Enable closed-loop control')
        self._ctrl_enable_cb.setChecked(False)
        en_row.addWidget(self._ctrl_enable_cb)
        en_row.addStretch()
        cl_lay.addLayout(en_row)

        gain_row = QtWidgets.QHBoxLayout()
        gain_row.addWidget(QtWidgets.QLabel('Loop gain:'))
        self._loop_gain_spin = QtWidgets.QDoubleSpinBox()
        self._loop_gain_spin.setRange(0.05, 1.00)
        self._loop_gain_spin.setSingleStep(0.05)
        self._loop_gain_spin.setDecimals(2)
        self._loop_gain_spin.setValue(0.50)
        self._loop_gain_spin.setToolTip(
            f'Fraction of the dB error applied per control update '
            f'(one per {CTRL_WINDOW_S:.0f} s of fresh data).\n'
            'Error decays as (1 - gain)^n, so 0.5 reaches 97% in 5 updates '
            f'(~{5*CTRL_WINDOW_S:.0f} s).\n'
            '1.0 is deadbeat — fastest, but passes all measurement noise '
            'into the drive.\n'
            '0.4-0.6 is the sweet spot.'
        )
        gain_row.addWidget(self._loop_gain_spin)
        gain_row.addStretch()
        gain_row.addWidget(QtWidgets.QLabel('Max corr:'))
        self._max_corr_spin = QtWidgets.QSpinBox()
        self._max_corr_spin.setRange(3, 200)
        self._max_corr_spin.setValue(20)
        self._max_corr_spin.setSuffix(' dB')
        self._max_corr_spin.setToolTip(
            'Per-bin correction clamp — a ceiling, not a target. The loop uses\n'
            'only what it needs; this bounds how far one bin may be pushed.\n'
            '\n'
            'Raising it costs drive headroom: the block is normalised to unit\n'
            'RMS, so a bin boosted by X dB pulls every other bin down. A bin\n'
            'pinned at +120 dB takes essentially all the power and starves the\n'
            'part of the band that was working.\n'
            '\n'
            f'This sets the BOOST limit. Cutting is capped separately at\n'
            f'−{CTRL_MAX_CUT_DB:.0f} dB: past that a bin is already switched off, so if\n'
            'it is still too loud the energy is cross-coupled from a resonance\n'
            'or a harmonic, and cutting further cannot null it.\n'
            '\n'
            'Watch the "sat" figure next to Error RMS — it is the share of\n'
            'in-band bins sitting on either rail. Persistently non-zero means\n'
            'those frequencies are past what the rig can do, and narrowing the\n'
            'profile will converge better than raising this further.'
        )
        gain_row.addWidget(self._max_corr_spin)
        cl_lay.addLayout(gain_row)

        self._spec_status_lbl = QtWidgets.QLabel('—')
        self._spec_status_lbl.setAlignment(QtCore.Qt.AlignCenter)
        self._spec_status_lbl.setStyleSheet(
            'font-family: Consolas; font-weight: bold; color: #aaa; '
            'border: 1px solid #555; padding: 3px;')
        self._spec_status_lbl.setToolTip(
            f'Share of in-band bins outside ±{TOL_ALARM_DB:.0f} dB (alarm) and '
            f'±{TOL_ABORT_DB:.0f} dB (abort) of the demand profile.'
        )
        cl_lay.addWidget(self._spec_status_lbl)

        err_row = QtWidgets.QHBoxLayout()
        self._ctrl_err_lbl = QtWidgets.QLabel('Error RMS: —')
        self._ctrl_err_lbl.setStyleSheet('font-family: Consolas; color: #aaa;')
        reset_btn = QtWidgets.QPushButton('↺ Reset')
        reset_btn.setFixedWidth(70)
        reset_btn.setToolTip(
            'Discard what the loop has learned since the saved response.\n'
            'Returns to the saved speaker response, not to flat.'
        )
        reset_btn.clicked.connect(self._on_ctrl_reset)
        err_row.addWidget(self._ctrl_err_lbl)
        err_row.addStretch()
        err_row.addWidget(reset_btn)
        cl_lay.addLayout(err_row)

        resp_row = QtWidgets.QHBoxLayout()
        resp_row.addWidget(QtWidgets.QLabel('Speaker response:'))
        resp_row.addStretch()
        save_resp_btn = QtWidgets.QPushButton('Save')
        save_resp_btn.setFixedWidth(60)
        save_resp_btn.setToolTip(
            'Store the current correction as this rig\'s speaker response.\n'
            'Reloaded automatically at startup, so the loop begins from the\n'
            'learned inverse transfer function instead of flat.\n'
            'Do this once the error has converged.'
        )
        save_resp_btn.clicked.connect(self.save_response_requested)
        load_resp_btn = QtWidgets.QPushButton('Load')
        load_resp_btn.setFixedWidth(60)
        load_resp_btn.setToolTip('Re-apply the saved speaker response from disk.')
        load_resp_btn.clicked.connect(self.load_response_requested)
        resp_row.addWidget(save_resp_btn)
        resp_row.addWidget(load_resp_btn)
        cl_lay.addLayout(resp_row)

        vbox.addWidget(cl_grp)

        # ── Audio Output ──────────────────────────────────────────────────
        audio_grp = QtWidgets.QGroupBox(
            'Audio Output' if HAS_SOUNDDEVICE else 'Audio Output  (install sounddevice)'
        )
        audio_grp.setEnabled(HAS_SOUNDDEVICE)
        audio_lay = QtWidgets.QVBoxLayout(audio_grp)

        dev_row = QtWidgets.QHBoxLayout()
        dev_row.addWidget(QtWidgets.QLabel('Device:'))
        self._audio_dev_combo = QtWidgets.QComboBox()
        self._audio_dev_combo.setMinimumWidth(130)
        dev_row.addWidget(self._audio_dev_combo, stretch=1)
        refresh_audio_btn = QtWidgets.QPushButton('↺')
        refresh_audio_btn.setFixedWidth(28)
        refresh_audio_btn.clicked.connect(self._refresh_audio_devices)
        dev_row.addWidget(refresh_audio_btn)
        audio_lay.addLayout(dev_row)

        rate_row = QtWidgets.QHBoxLayout()
        rate_row.addWidget(QtWidgets.QLabel('Rate:'))
        self._audio_rate_combo = QtWidgets.QComboBox()
        for r in ('44100', '48000', '96000'):
            self._audio_rate_combo.addItem(r)
        rate_row.addWidget(self._audio_rate_combo)
        rate_row.addStretch()
        audio_lay.addLayout(rate_row)

        level_row = QtWidgets.QHBoxLayout()
        level_row.addWidget(QtWidgets.QLabel('Level:'))
        self._audio_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._audio_slider.setRange(-40, 0)
        self._audio_slider.setValue(-20)          # -20 dB default
        self._audio_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self._audio_slider.setTickInterval(10)
        self._audio_slider.setToolTip(
            'Starting drive level. With the loop enabled this is a starting\n'
            'point, not a fixed setting — the level servo trims from here to\n'
            'match the measured Grms to demand. The live value is shown to the\n'
            'right; move the slider at any time to re-seed it.'
        )
        self._audio_slider.valueChanged.connect(self._on_audio_level_changed)
        self._level_db: float = float(self._audio_slider.value())
        self._audio_level_lbl = QtWidgets.QLabel('-20 dB')
        self._audio_level_lbl.setStyleSheet('font-family: Consolas; color: #aaa;')
        self._audio_level_lbl.setFixedWidth(55)
        level_row.addWidget(self._audio_slider, stretch=1)
        level_row.addWidget(self._audio_level_lbl)
        audio_lay.addLayout(level_row)

        audio_btn_row = QtWidgets.QHBoxLayout()
        self._audio_start_btn = QtWidgets.QPushButton('▶  Drive On')
        self._audio_stop_btn  = QtWidgets.QPushButton('■  Drive Off')
        self._audio_stop_btn.setEnabled(False)
        self._audio_start_btn.clicked.connect(self._on_audio_start)
        self._audio_stop_btn.clicked.connect(self._on_audio_stop)
        self._audio_clip_lbl = QtWidgets.QLabel('')
        self._audio_clip_lbl.setStyleSheet('color: #f44; font-weight: bold;')
        audio_btn_row.addWidget(self._audio_start_btn)
        audio_btn_row.addWidget(self._audio_stop_btn)
        audio_btn_row.addStretch()
        audio_btn_row.addWidget(self._audio_clip_lbl)
        audio_lay.addLayout(audio_btn_row)

        self._clip_clear_timer = QtCore.QTimer(self)
        self._clip_clear_timer.setSingleShot(True)
        self._clip_clear_timer.setInterval(2000)
        self._clip_clear_timer.timeout.connect(lambda: self._audio_clip_lbl.setText(''))

        vbox.addWidget(audio_grp)
        vbox.addStretch()

        self._update_grms_base_label()
        self._update_total_label()
        if HAS_SOUNDDEVICE:
            self._refresh_audio_devices()

    # ── Breakpoint helpers ────────────────────────────────────────────────────

    def _add_bp_row(self) -> None:
        self._bp_model.add_row()

    def _del_bp_row(self) -> None:
        sel = self._bp_view.selectionModel().currentIndex()
        if sel.isValid():
            self._bp_model.remove_row(sel.row())

    def _on_profile_changed(self) -> None:
        self._update_grms_base_label()
        self._push_audio_profile()
        self.profile_changed.emit()

    def _update_grms_base_label(self) -> None:
        g = psd_grms(self._bp_model.breakpoints(), 0.0)
        self._grms_base_lbl.setText(f'Base Grms: {g:.3f} g')

    def breakpoints(self) -> list[tuple[float, float]]:
        return self._bp_model.breakpoints()

    # ── Sequence helpers ──────────────────────────────────────────────────────

    def _add_seq_row(self) -> None:
        self._seq_model.add_row()

    def _del_seq_row(self) -> None:
        sel = self._seq_view.selectionModel().currentIndex()
        if sel.isValid():
            self._seq_model.remove_row(sel.row())

    def _update_total_label(self) -> None:
        t = self._seq_model.total_duration()
        m, s = divmod(int(t), 60)
        self._total_lbl.setText(f'Total: {m}m {s:02d}s')

    # ── Channel helpers ───────────────────────────────────────────────────────

    def control_channels(self) -> list[str]:
        return [n for n, cb in self._ch_checks.items() if cb.isChecked()]

    # ── Test runner ───────────────────────────────────────────────────────────

    def _start_test(self) -> None:
        if not self._seq_model.steps():
            return
        self._running   = True
        self._paused    = False
        self._step_idx  = 0
        self._elapsed_s = 0.0
        self._start_btn.setEnabled(False)
        self._pause_btn.setEnabled(True)
        self._stop_btn.setEnabled(True)
        self.test_started.emit()                     # → MainWindow resets correction
        self._go_to_step(0)                          # sets step_idx before audio starts
        if HAS_SOUNDDEVICE and self._audio_worker is None:
            self._on_audio_start()                   # auto-start drive at step-0 gain
        else:
            self._push_audio_profile()               # already running — update gain now
        self._timer.start()

    def _pause_test(self) -> None:
        if not self._running:
            return
        self._paused = not self._paused
        if self._paused:
            self._timer.stop()
            self._pause_btn.setText('▶  Resume')
            self._step_lbl.setStyleSheet('color: #fa0; font-family: Consolas;')
        else:
            self._timer.start()
            self._pause_btn.setText('⏸  Pause')
            self._step_lbl.setStyleSheet('color: #4f4; font-family: Consolas;')

    def _stop_test(self) -> None:
        self._timer.stop()
        self._running = False
        self._paused  = False
        self._start_btn.setEnabled(True)
        self._pause_btn.setEnabled(False)
        self._pause_btn.setText('⏸  Pause')
        self._stop_btn.setEnabled(False)
        self._step_lbl.setText('Stopped')
        self._step_lbl.setStyleSheet('color: #888; font-family: Consolas;')
        self._progress.setValue(0)
        self.test_stopped.emit()

    def _go_to_step(self, idx: int) -> None:
        steps = self._seq_model.steps()
        if idx >= len(steps):
            self._complete_test()
            return
        self._step_idx  = idx
        self._elapsed_s = 0.0
        _, gain = steps[idx]
        self.step_started.emit(idx, gain)
        self._push_audio_profile()
        self._update_step_display()

    def _tick(self) -> None:
        steps = self._seq_model.steps()
        if self._step_idx >= len(steps):
            return
        self._elapsed_s += 1.0
        dur, _ = steps[self._step_idx]
        if self._elapsed_s >= dur:
            self._go_to_step(self._step_idx + 1)
        else:
            self._update_step_display()

    def _update_step_display(self) -> None:
        steps = self._seq_model.steps()
        if self._step_idx >= len(steps):
            return
        dur, gain = steps[self._step_idx]
        remaining = int(dur - self._elapsed_s)
        m, s = divmod(remaining, 60)
        target_g = psd_grms(self._bp_model.breakpoints(), gain)
        self._step_lbl.setText(
            f'Step {self._step_idx+1}/{len(steps)}  ·  {gain:+.1f} dB  ·  '
            f'{target_g:.3f} g  ·  {m}:{s:02d} left'
        )
        if not self._paused:
            self._step_lbl.setStyleSheet('color: #4f4; font-family: Consolas;')
        self._progress.setValue(int(100 * self._elapsed_s / dur))
        self._grms_target_lbl.setText(f'Target: {target_g:.3f} g')
        self._grms_target_lbl.setStyleSheet('color: #fff; font-family: Consolas;')

    def _complete_test(self) -> None:
        self._timer.stop()
        self._running = False
        self._start_btn.setEnabled(True)
        self._pause_btn.setEnabled(False)
        self._stop_btn.setEnabled(False)
        self._step_lbl.setText('Complete ✓')
        self._step_lbl.setStyleSheet('color: #4f4; font-family: Consolas;')
        self._progress.setValue(100)
        self._push_audio_profile()   # reverts to 0 dB on the audio thread
        self.test_stopped.emit()

    # ── Audio helpers ─────────────────────────────────────────────────────────

    def _refresh_audio_devices(self) -> None:
        if not HAS_SOUNDDEVICE:
            return
        self._audio_dev_combo.clear()
        self._audio_dev_combo.addItem('Default', userData=-1)
        try:
            for i, d in enumerate(_sd.query_devices()):
                if d['max_output_channels'] > 0:
                    self._audio_dev_combo.addItem(d['name'], userData=i)
        except Exception:
            pass

    def _on_audio_level_changed(self, db_val: int) -> None:
        # Moving the slider re-seeds the servo rather than fighting it.
        self._level_db = float(db_val)
        self._apply_level()

    def _apply_level(self) -> None:
        self._level_db = max(LEVEL_MIN_DB, min(0.0, self._level_db))
        self._audio_level_lbl.setText(f'{self._level_db:+.1f} dB')
        if self._audio_worker is not None:
            self._audio_worker.set_output_gain(10.0 ** (self._level_db / 20.0))

    @property
    def level_db(self) -> float:
        return self._level_db

    def nudge_level_db(self, delta_db: float) -> None:
        """Level servo. The per-bin correction cannot change overall level —
        unit-RMS normalisation divides any common-mode part straight out — so
        this is the only path that can."""
        if abs(delta_db) < 1e-3:
            return
        self._level_db += delta_db
        self._apply_level()

    def _on_audio_start(self) -> None:
        if self._audio_worker is not None:
            return
        w = AudioOutputWorker()
        w.set_profile(self._bp_model.breakpoints(), self.current_gain_db())
        w.set_output_gain(10.0 ** (self._level_db / 20.0))
        dev_idx = self._audio_dev_combo.currentData()
        if dev_idx is not None:
            w.set_device(int(dev_idx))
        fs_text = self._audio_rate_combo.currentText()
        w.set_fs(int(fs_text))
        w.clip_detected.connect(self._on_clip_detected)
        w.start()
        self._audio_worker = w
        self._audio_start_btn.setEnabled(False)
        self._audio_stop_btn.setEnabled(True)
        self._audio_clip_lbl.setText('')
        # Hand the worker whatever correction is already learned/loaded —
        # otherwise a saved response is ignored until the next Welch cycle,
        # and never applied at all when the loop is disabled.
        self.audio_started.emit()

    def _on_audio_stop(self) -> None:
        if self._audio_worker is None:
            return
        self._audio_worker.stop()
        self._audio_worker = None
        self._audio_start_btn.setEnabled(True)
        self._audio_stop_btn.setEnabled(False)
        self._audio_clip_lbl.setText('')

    def _on_clip_detected(self) -> None:
        self._audio_clip_lbl.setText('CLIP')
        self._clip_clear_timer.start()

    # ── Control loop helpers ──────────────────────────────────────────────────

    @property
    def loop_enabled(self) -> bool:
        return self._ctrl_enable_cb.isChecked()

    @property
    def loop_gain(self) -> float:
        return self._loop_gain_spin.value()

    @property
    def max_correction_db(self) -> float:
        return float(self._max_corr_spin.value())

    def push_correction(self, freqs: np.ndarray, corr_db: np.ndarray) -> None:
        if self._audio_worker is not None:
            self._audio_worker.set_correction(freqs, corr_db)

    def clear_correction(self) -> None:
        if self._audio_worker is not None:
            self._audio_worker.clear_correction()

    def update_loop_error(self, rms_db: float, sat_frac: float = 0.0) -> None:
        txt = f'Error RMS: {rms_db:.2f} dB'
        if sat_frac > 0.0:
            # Bins pinned on the clamp: the rig cannot reach demand there, and
            # they are eating drive headroom from the bins that can.
            txt += f'  ·  sat {sat_frac * 100:.0f}%'
        self._ctrl_err_lbl.setText(txt)
        if rms_db < 3.0:
            colour = '#4f4'   # converged
        elif rms_db < 10.0:
            colour = '#fa0'   # converging
        else:
            colour = '#f44'   # diverged / not started
        if sat_frac > 0.05:
            colour = '#f44'   # clamp is binding — flag it regardless of error
        self._ctrl_err_lbl.setStyleSheet(f'font-family: Consolas; color: {colour};')

    def update_spec_status(self, alarm_frac: float, abort_frac: float) -> None:
        """In-spec readout: share of controlled bins outside each tolerance."""
        if abort_frac > 0.02:
            txt, colour = 'ABORT', '#f44'
        elif alarm_frac > 0.05:
            txt, colour = 'ALARM', '#fa0'
        else:
            txt, colour = 'IN SPEC', '#4f4'
        self._spec_status_lbl.setText(
            f'{txt}   ±{TOL_ALARM_DB:.0f}dB {(1-alarm_frac)*100:.0f}%   '
            f'±{TOL_ABORT_DB:.0f}dB {(1-abort_frac)*100:.0f}%')
        self._spec_status_lbl.setStyleSheet(
            f'font-family: Consolas; font-weight: bold; color: {colour}; '
            f'border: 1px solid {colour}; padding: 3px;')

    def _on_ctrl_reset(self) -> None:
        self.clear_correction()
        self._spec_status_lbl.setText('—')
        self._spec_status_lbl.setStyleSheet(
            'font-family: Consolas; font-weight: bold; color: #aaa; '
            'border: 1px solid #555; padding: 3px;')
        self._ctrl_err_lbl.setText('Error RMS: —')
        self._ctrl_err_lbl.setStyleSheet('font-family: Consolas; color: #aaa;')
        # Signal MainWindow to zero its correction array too
        self.profile_changed.emit()

    def _push_audio_profile(self) -> None:
        if self._audio_worker is not None:
            self._audio_worker.set_profile(
                self._bp_model.breakpoints(), self.current_gain_db()
            )

    def current_gain_db(self) -> float:
        if not self._running:
            return 0.0
        steps = self._seq_model.steps()
        return steps[self._step_idx][1] if self._step_idx < len(steps) else 0.0

    @property
    def is_running(self) -> bool:
        return self._running

    def test_elapsed_s(self) -> float:
        """Elapsed across the whole sequence — _elapsed_s alone is per-step."""
        steps = self._seq_model.steps()
        done  = sum(d for d, _ in steps[:self._step_idx])
        return float(done + self._elapsed_s)

    def demand_grms_trace(self) -> tuple[np.ndarray, np.ndarray]:
        """Target Grms staircase for the entire sequence, computed up front.

        Each step's Grms is the analytic integral of the profile at that step's
        gain, so the whole demand history is known before the test starts.
        """
        bp = self._bp_model.breakpoints()
        ts: list[float] = []
        gs: list[float] = []
        t = 0.0
        for dur, gain in self._seq_model.steps():
            g = psd_grms(bp, gain)
            ts.extend((t, t + dur))    # flat within the step, vertical at the edge
            gs.extend((g, g))
            t += dur
        return np.asarray(ts, dtype=np.float64), np.asarray(gs, dtype=np.float64)

    def update_measured_grms(self, grms: float) -> None:
        self._grms_meas_lbl.setText(f'Meas: {grms:.3f} g')
        self._grms_meas_lbl.setStyleSheet(
            'color: #4f4; font-family: Consolas;' if grms > 0 else 'color: #aaa; font-family: Consolas;'
        )


# ── Main window ───────────────────────────────────────────────────────────────

class MainWindow(QtWidgets.QMainWindow):

    _fft_done = QtCore.pyqtSignal(object)

    def __init__(self, demo: bool = False) -> None:
        super().__init__()
        self.demo   = demo
        self.worker: Optional[SerialWorker | DemoWorker] = None

        self._sensitivity  = FSR_OPTIONS[FSR_DEFAULT]
        self._total_drops  = 0

        self._ring:      np.ndarray = np.zeros((HISTORY, CHANNEL_COUNT), dtype=np.float32)
        self._ring_filt: np.ndarray = np.zeros((HISTORY, CHANNEL_COUNT), dtype=np.float32)
        self._disp:      np.ndarray = np.zeros((HISTORY, CHANNEL_COUNT), dtype=np.float32)
        self._ring_ptr:    int        = 0
        self._n_samples:   int        = 0
        self._plot_samples: int       = int(2.0 * SAMPLE_RATE)  # initial display: 2 s
        self._t_axis:      np.ndarray = np.linspace(-WINDOW_TIME, 0.0, HISTORY, dtype=np.float32)

        self._fs_meas:    float       = SAMPLE_RATE
        self._fs_history: list[float] = []

        self._sos, self._zi = self._build_filter(SAMPLE_RATE)
        self._update_freq_arrays(SAMPLE_RATE)
        # Control loop correction spectrum (dB power, 0 = no correction).
        # Shape matches _plot_freqs. _H_base_db is the saved rig response that
        # Reset falls back to; _H_corr_db is what is actually applied.
        self._spec_floor_log: float = PSD_FLOOR_LOG
        self._spec_base_lo:   float = PSD_FLOOR_LOG
        self._spec_hi:        float = 0.0
        self._spec_band_mask: Optional[np.ndarray] = None
        self._H_base_db: np.ndarray = np.zeros(len(self._plot_freqs), dtype=np.float64)
        self._H_corr_db: np.ndarray = np.zeros(len(self._plot_freqs), dtype=np.float64)

        self._psd:         Optional[list[np.ndarray]] = None   # EMA, display only
        self._fft_running: bool = False
        self._last_welch_n: int = 0   # gates display Welch rate
        self._last_ctrl_n:  int = 0   # gates control updates (fresh windows)

        # Grms timeline history (test time in s, measured Grms in g)
        self._grms_t: list[float] = []
        self._grms_v: list[float] = []

        pool = QtCore.QThreadPool.globalInstance()
        assert pool is not None
        self._pool: QtCore.QThreadPool = pool

        self._build_ui()

        # Initialise target curve and demand Grms timeline with default profile
        self._recompute_target()
        self._rebuild_grms_demand()

        self._fs_lbl.setText(f'FS: {SAMPLE_RATE:.0f} Hz (configured)')
        self._fft_done.connect(self._on_fft_done)

        # Instantiate the status bar up-front so later messages don't reflow the layout.
        self.statusBar().showMessage('Ready')

        # Start from the rig's previously learned response, if we have one.
        self._load_response(announce=False)
        if np.any(self._H_base_db):
            self.statusBar().showMessage(
                f'Speaker response loaded from {RESPONSE_FILE.name} '
                f'(peak {np.max(np.abs(self._H_base_db)):.1f} dB)', 8000)

        self._display_timer = QtCore.QTimer(self)
        self._display_timer.setInterval(33)
        self._display_timer.timeout.connect(self._refresh_display)
        self._display_timer.start()

        if demo:
            self._start_worker(DemoWorker())

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.setWindowTitle('IMU Visualizer — ICM-42688-P')
        self.resize(1600, 900)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setSpacing(4)
        root.setContentsMargins(6, 6, 6, 6)
        root.addLayout(self._build_toolbar())
        root.addWidget(self._build_plot_panel(), stretch=1)

        # Test profile dock — right side
        self._profile_dock = TestProfileDock(self)
        self._profile_dock.setMinimumWidth(290)
        self._profile_dock.setMaximumWidth(360)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self._profile_dock)

        self._profile_dock.profile_changed.connect(self._recompute_target)
        self._profile_dock.profile_changed.connect(self._reset_correction)
        self._profile_dock.profile_changed.connect(self._rebuild_grms_demand)
        self._profile_dock.step_started.connect(self._on_step_started)
        self._profile_dock.test_started.connect(self._reset_correction)
        self._profile_dock.test_started.connect(self._on_test_started)
        self._profile_dock.test_stopped.connect(self._on_test_stopped)
        self._profile_dock.save_response_requested.connect(self._save_response)
        self._profile_dock.load_response_requested.connect(self._load_response)
        self._profile_dock.audio_started.connect(self._push_current_correction)

    def _build_toolbar(self) -> QtWidgets.QHBoxLayout:
        bar = QtWidgets.QHBoxLayout()

        self._port_combo = QtWidgets.QComboBox()
        self._port_combo.setMinimumWidth(120)
        self._refresh_ports()

        refresh_btn = QtWidgets.QPushButton('↺')
        refresh_btn.setFixedWidth(30)
        refresh_btn.setToolTip('Refresh port list')
        refresh_btn.clicked.connect(self._refresh_ports)

        self._baud_combo = QtWidgets.QComboBox()
        for b in ('115200', '230400', '921600', '1000000'):
            self._baud_combo.addItem(b)
        self._baud_combo.setCurrentText('115200')

        self._fsr_combo = QtWidgets.QComboBox()
        for label in FSR_OPTIONS:
            self._fsr_combo.addItem(label)
        self._fsr_combo.setCurrentText(FSR_DEFAULT)
        self._fsr_combo.setToolTip('Must match ICM_ACCEL_CONFIG in firmware — reconnect to apply')

        self._connect_btn = QtWidgets.QPushButton('Connect')
        self._connect_btn.setFixedWidth(95)
        self._connect_btn.clicked.connect(self._toggle_connection)
        if self.demo:
            self._connect_btn.setEnabled(False)

        self._status_lbl = QtWidgets.QLabel('●  Disconnected')
        self._status_lbl.setStyleSheet('color: #888;')

        self._fs_lbl = QtWidgets.QLabel('FS: — Hz')
        self._fs_lbl.setStyleSheet('color: #aaa;')
        f = self._fs_lbl.font(); f.setFamily('Consolas'); self._fs_lbl.setFont(f)

        self._drop_lbl = QtWidgets.QLabel('Drops: 0')
        self._drop_lbl.setStyleSheet('color: #aaa;')
        f2 = self._drop_lbl.font(); f2.setFamily('Consolas'); self._drop_lbl.setFont(f2)

        self._plot_len_edit = QtWidgets.QLineEdit('2')
        self._plot_len_edit.setFixedWidth(55)
        self._plot_len_edit.setToolTip(
            f'Time-domain plot length in seconds (press Enter, max {WINDOW_TIME:.0f} s). '
            'FFT always uses the full ring buffer.'
        )
        self._plot_len_edit.returnPressed.connect(self._on_plot_len_edit)

        for w in (
            QtWidgets.QLabel('Port:'), self._port_combo, refresh_btn,
            QtWidgets.QLabel('Baud:'), self._baud_combo,
            QtWidgets.QLabel('FSR:'),  self._fsr_combo,
            self._connect_btn, self._status_lbl, self._fs_lbl, self._drop_lbl,
            QtWidgets.QLabel('Plot (s):'), self._plot_len_edit,
        ):
            bar.addWidget(w)
        bar.addStretch()
        return bar

    def _on_plot_len_edit(self) -> None:
        try:
            secs = float(self._plot_len_edit.text())
        except ValueError:
            return
        secs = max(0.0001, min(secs, WINDOW_TIME))
        self._plot_samples = int(secs * SAMPLE_RATE)
        self._plot.setXRange(-secs, 0.0)

    def _build_plot_panel(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        cb_box  = QtWidgets.QGroupBox('Visible Traces')
        cb_grid = QtWidgets.QGridLayout(cb_box)
        cb_grid.setVerticalSpacing(2)
        cb_grid.setHorizontalSpacing(10)

        self._plot = pg.PlotWidget()
        self._plot.setLabel('left', 'Acceleration (g)')
        self._plot.setLabel('bottom', 'Time (s)')
        self._plot.showGrid(x=True, y=True, alpha=0.2)
        self._plot.setDownsampling(mode='peak')
        self._plot.setClipToView(True)
        self._plot.addLegend(offset=(8, 8))

        self._spec_plot = pg.PlotWidget(
            axisItems={'bottom': LogHzAxis(orientation='bottom'),
                       'left':   LogPSDAxis(orientation='left')}
        )
        self._spec_plot.setLabel('left', 'PSD  (g²/Hz)')
        self._spec_plot.setLabel('bottom', 'Frequency  (Hz)')
        self._spec_plot.showGrid(x=True, y=True, alpha=0.2)
        self._spec_plot.addLegend(offset=(8, 8))
        self._spec_plot.getAxis('left').enableAutoSIPrefix(False)
        self._spec_plot.setXRange(
            math.log10(PSD_FMIN), math.log10(PSD_FMAX), padding=0
        )

        self._curves:      dict[str, pg.PlotDataItem] = {}
        self._spec_curves: dict[str, pg.PlotDataItem] = {}

        _init_t = self._t_axis[-self._plot_samples:]
        for idx, (name, color) in enumerate(CHANNEL_DICT.items()):
            self._curves[name] = self._plot.plot(
                _init_t, np.zeros(self._plot_samples), name=name,
                pen=pg.mkPen(color=color, width=1.5),
            )
            self._curves[name].setDownsampling(ds=True, auto=True, method='peak')
            self._curves[name].setClipToView(True)

            self._spec_curves[name] = self._spec_plot.plot(
                self._log_plot_freqs,
                np.full(len(self._log_plot_freqs), PSD_FLOOR_LOG),
                name=name, pen=pg.mkPen(color=color, width=1.5),
            )

            cb = QtWidgets.QCheckBox(name)
            cb.setChecked(True)
            cb.setStyleSheet(f'color: rgb{color};')
            cb.stateChanged.connect(
                lambda state, n=name: self._set_trace_visible(n, bool(state))
            )
            cb_grid.addWidget(cb, 0, idx)

        # Tolerance bands, drawn under the target so the target stays readable.
        # Highlight the profile band. Shading *outside* it would be invisible —
        # the background is already black — so tint the controlled band instead.
        self._band_shade = pg.LinearRegionItem(
            values=(0, 0), movable=False,
            brush=pg.mkBrush(70, 110, 150, 38),
            pen=pg.mkPen(color=(120, 140, 160), width=1,
                         style=QtCore.Qt.DashLine),
        )
        self._band_shade.setZValue(-10)
        self._spec_plot.addItem(self._band_shade)

        blank = np.full(len(self._log_plot_freqs), PSD_FLOOR_LOG)
        self._tol_curves: list[pg.PlotDataItem] = []
        for tol, colour, style in (
                (TOL_ABORT_DB, (235,  80,  80), QtCore.Qt.DashLine),
                (TOL_ALARM_DB, (235, 190,  70), QtCore.Qt.DotLine)):
            for sign in (+1, -1):
                c = self._spec_plot.plot(
                    self._log_plot_freqs, blank,
                    name=(f'±{tol:.0f} dB' if sign > 0 else None),
                    pen=pg.mkPen(color=colour, width=1.6, style=style),
                )
                c.setVisible(False)
                self._tol_curves.append(c)

        # Target PSD overlay — dashed white
        self._target_curve = self._spec_plot.plot(
            self._log_plot_freqs, blank,
            name='Target',
            pen=pg.mkPen(color=(220, 220, 220), width=2.0,
                         style=QtCore.Qt.DashLine),
        )

        # Correction H(f) on its own right-hand dB axis. It is not a PSD — the
        # drive's absolute level is set by the Level slider after unit-RMS
        # normalisation — so sharing the left axis would be misleading, and
        # forcing it into view would wreck the tolerance-band scaling.
        spec_item = self._spec_plot.getPlotItem()
        self._corr_vb = pg.ViewBox()
        spec_item.showAxis('right')
        spec_item.scene().addItem(self._corr_vb)
        spec_item.getAxis('right').linkToView(self._corr_vb)
        self._corr_vb.setXLink(spec_item)
        spec_item.getAxis('right').setLabel('Correction  (dB)')
        spec_item.getAxis('right').setPen(pg.mkPen(color=(80, 230, 80)))
        spec_item.getAxis('right').setTextPen(pg.mkPen(color=(80, 230, 80)))

        def _sync_corr_vb() -> None:
            self._corr_vb.setGeometry(spec_item.vb.sceneBoundingRect())
            self._corr_vb.linkedViewChanged(spec_item.vb, self._corr_vb.XAxis)

        spec_item.vb.sigResized.connect(_sync_corr_vb)
        _sync_corr_vb()

        self._drive_curve = pg.PlotDataItem(
            pen=pg.mkPen(color=(80, 230, 80), width=1.8,
                         style=QtCore.Qt.DotLine))
        self._corr_vb.addItem(self._drive_curve)
        self._drive_curve.setVisible(False)
        # Legend lives on the main plot, so register a proxy entry for it.
        self._spec_plot.plot([], [], name='Correction →',
                             pen=pg.mkPen(color=(80, 230, 80), width=1.8,
                                          style=QtCore.Qt.DotLine))

        # ── Grms timeline: precomputed demand staircase vs live measured ──────
        self._grms_plot = pg.PlotWidget()
        self._grms_plot.setLabel('left', 'Grms  (g)')
        self._grms_plot.setLabel('bottom', 'Test time  (s)')
        self._grms_plot.showGrid(x=True, y=True, alpha=0.2)
        self._grms_plot.addLegend(offset=(8, 8))
        self._grms_plot.getAxis('left').enableAutoSIPrefix(False)

        self._grms_tol_curves = [
            self._grms_plot.plot(
                [], [], name=(f'±{TOL_ALARM_DB:.0f} dB' if s > 0 else None),
                pen=pg.mkPen(color=(235, 190, 70), width=1.2,
                             style=QtCore.Qt.DotLine))
            for s in (+1, -1)
        ]
        self._grms_demand_curve = self._grms_plot.plot(
            [], [], name='Demand',
            pen=pg.mkPen(color=(220, 220, 220), width=2.0,
                         style=QtCore.Qt.DashLine),
        )
        self._grms_meas_curve = self._grms_plot.plot(
            [], [], name='Measured',
            pen=pg.mkPen(color=(80, 230, 80), width=1.8),
        )
        self._grms_now_line = pg.InfiniteLine(
            pos=0.0, angle=90, movable=False,
            pen=pg.mkPen(color=(120, 200, 255), width=1,
                         style=QtCore.Qt.DashLine))
        self._grms_now_line.setVisible(False)
        self._grms_plot.addItem(self._grms_now_line)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        splitter.addWidget(self._plot)
        splitter.addWidget(self._spec_plot)
        splitter.addWidget(self._grms_plot)
        splitter.setSizes([280, 440, 240])   # spectrum is the money plot

        layout.addWidget(cb_box)
        layout.addWidget(splitter, stretch=1)
        return widget

    def _set_trace_visible(self, name: str, visible: bool) -> None:
        self._curves[name].setVisible(visible)
        self._spec_curves[name].setVisible(visible)

    # ── Spectral helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _build_filter(fs: float):
        hp_freq = PSD_FMIN
        lp_freq = min(PSD_FMAX - 5.0, fs * 0.49)
        sos      = butter(4, [hp_freq, lp_freq], btype='bandpass', fs=fs, output='sos')
        zi_proto = sosfilt_zi(sos)
        zi       = np.stack([zi_proto] * CHANNEL_COUNT, axis=-1)
        return sos, zi

    def _update_freq_arrays(self, fs: float) -> None:
        freqs            = np.fft.rfftfreq(SPEC_N, 1.0 / fs)
        mask             = (freqs >= PSD_FMIN) & (freqs <= PSD_FMAX)
        self._plot_freqs = freqs[mask]
        self._plot_mask  = mask
        self._log_plot_freqs = np.log10(self._plot_freqs)

    def _update_fs(self, fs: float) -> None:
        if abs(fs - self._fs_meas) / self._fs_meas < 0.02:
            return
        self._fs_meas       = fs
        self._sos, self._zi = self._build_filter(fs)
        self._update_freq_arrays(fs)
        self._fs_lbl.setText(f'FS: {fs:.0f} Hz (measured)')
        self._fs_lbl.setStyleSheet('color: #4f4;')

    def _recompute_target(self) -> None:
        bp      = self._profile_dock.breakpoints()
        gain_db = self._profile_dock.current_gain_db()
        psd_vals = psd_interp_loglog(self._plot_freqs, bp, gain_db)
        self._target_curve.setData(self._log_plot_freqs, np.log10(psd_vals))

        # Anchor the y-range to the profile. Without this a single bin sitting on
        # the correction clamp (down to demand × 10^-12 at 120 dB) drags autoscale
        # to 1e-15 and squashes every real trace into the top of the pane.
        lo = np.log10(psd_vals.min()) + PSD_VIEW_DECADES_BELOW
        hi = np.log10(psd_vals.max()) + PSD_VIEW_DECADES_ABOVE
        # Baseline; _refresh_display may drop the floor further to reveal a
        # badly off-target trace, bounded by PSD_VIEW_MAX_DECADES.
        self._spec_base_lo   = lo
        self._spec_hi        = hi
        self._spec_floor_log = min(self._spec_floor_log, lo) \
            if self._spec_floor_log > PSD_FLOOR_LOG else lo
        self._spec_floor_log = max(self._spec_floor_log, hi - PSD_VIEW_MAX_DECADES)
        self._spec_plot.setYRange(self._spec_floor_log, hi, padding=0)

        # Tolerance bands, drawn only across the band the loop actually controls.
        band     = (self._plot_freqs >= bp[0][0]) & (self._plot_freqs <= bp[-1][0])
        self._spec_band_mask = band
        base_log = np.log10(psd_vals)[band]
        lf_band  = self._log_plot_freqs[band]
        offsets  = (TOL_ABORT_DB, -TOL_ABORT_DB, TOL_ALARM_DB, -TOL_ALARM_DB)
        for curve, off in zip(self._tol_curves, offsets):
            curve.setData(lf_band, base_log + off / 10.0)
            curve.setVisible(True)

        self._band_shade.setRegion(
            (math.log10(bp[0][0]), math.log10(bp[-1][0])))

    # ── Test runner callbacks ─────────────────────────────────────────────────

    @QtCore.pyqtSlot(int, float)
    def _on_step_started(self, _step_idx: int, _gain_db: float) -> None:
        self._recompute_target()

    def _rebuild_grms_demand(self) -> None:
        """Lay out the whole demand Grms history before the test runs."""
        t, g = self._profile_dock.demand_grms_trace()
        self._grms_demand_curve.setData(t, g)
        # Grms scales as sqrt(power), so a ±N dB band is ×10^(N/20).
        for curve, s in zip(self._grms_tol_curves, (+1, -1)):
            curve.setData(t, g * 10.0 ** (s * TOL_ALARM_DB / 20.0))
        if len(t):
            self._grms_plot.setXRange(0.0, float(t[-1]), padding=0.02)
            self._grms_plot.setYRange(
                0.0, float(np.max(g)) * 10 ** (TOL_ALARM_DB / 20.0) * 1.15,
                padding=0)

    @QtCore.pyqtSlot()
    def _on_test_started(self) -> None:
        self._grms_t.clear()
        self._grms_v.clear()
        self._grms_meas_curve.setData([], [])
        self._rebuild_grms_demand()

    @QtCore.pyqtSlot()
    def _on_test_stopped(self) -> None:
        self._recompute_target()   # revert to 0 dB overlay
        self._grms_now_line.setVisible(False)

    @QtCore.pyqtSlot()
    def _reset_correction(self) -> None:
        # Fall back to the saved speaker response, not to flat — the rig's
        # transfer function doesn't change between tests.
        self._H_corr_db = self._H_base_db.copy()
        if np.any(self._H_corr_db):
            self._profile_dock.push_correction(self._plot_freqs, self._H_corr_db)
        else:
            self._profile_dock.clear_correction()
        self._profile_dock._ctrl_err_lbl.setText('Error RMS: —')
        self._profile_dock._ctrl_err_lbl.setStyleSheet('font-family: Consolas; color: #aaa;')
        self._drive_curve.setVisible(False)

    @QtCore.pyqtSlot()
    def _push_current_correction(self) -> None:
        if (self._H_corr_db.shape == self._plot_freqs.shape
                and np.any(self._H_corr_db)):
            self._profile_dock.push_correction(self._plot_freqs, self._H_corr_db)

    # ── Speaker response persistence ──────────────────────────────────────────

    @QtCore.pyqtSlot()
    def _save_response(self) -> None:
        if self._H_corr_db.shape != self._plot_freqs.shape or not np.any(self._H_corr_db):
            QtWidgets.QMessageBox.information(
                self, 'Save speaker response',
                'No correction to save yet — run the loop until the error '
                'converges, then save.',
            )
            return
        try:
            RESPONSE_FILE.write_text(json.dumps({
                'note':     'Learned inverse response of the drive chain '
                            '(amp + speaker/shaker + fixture). dB power.',
                'fs_hz':    SAMPLE_RATE,
                'spec_n':   SPEC_N,
                'freqs_hz': [round(float(f), 3) for f in self._plot_freqs],
                'corr_db':  [round(float(c), 3) for c in self._H_corr_db],
            }, indent=1), encoding='utf-8')
        except OSError as exc:
            QtWidgets.QMessageBox.warning(
                self, 'Save speaker response', f'Could not write file:\n{exc}')
            return
        self._H_base_db = self._H_corr_db.copy()
        self.statusBar().showMessage(
            f'Speaker response saved to {RESPONSE_FILE.name} '
            f'(peak {np.max(np.abs(self._H_base_db)):.1f} dB)', 5000)

    @QtCore.pyqtSlot()
    def _load_response(self, announce: bool = True) -> None:
        if not RESPONSE_FILE.exists():
            if announce:
                QtWidgets.QMessageBox.information(
                    self, 'Load speaker response',
                    f'No saved response found at:\n{RESPONSE_FILE}')
            return
        try:
            data = json.loads(RESPONSE_FILE.read_text(encoding='utf-8'))
            f_saved = np.asarray(data['freqs_hz'], dtype=np.float64)
            c_saved = np.asarray(data['corr_db'],  dtype=np.float64)
        except (OSError, ValueError, KeyError) as exc:
            if announce:
                QtWidgets.QMessageBox.warning(
                    self, 'Load speaker response', f'Could not read file:\n{exc}')
            return
        if len(f_saved) < 2 or len(f_saved) != len(c_saved):
            return
        # Re-grid onto the current analysis bins; taper to 0 dB outside the
        # saved span so an old/narrower file can't inject edge-held boost.
        self._H_base_db = np.interp(
            np.log10(self._plot_freqs), np.log10(f_saved), c_saved,
            left=0.0, right=0.0,
        )
        self._H_corr_db = self._H_base_db.copy()
        self._profile_dock.push_correction(self._plot_freqs, self._H_corr_db)
        if announce:
            self.statusBar().showMessage(
                f'Speaker response loaded from {RESPONSE_FILE.name} '
                f'(peak {np.max(np.abs(self._H_base_db)):.1f} dB)', 5000)

    # ── Connection management ─────────────────────────────────────────────────

    def _refresh_ports(self) -> None:
        self._port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self._port_combo.addItem(p.device)

    def _toggle_connection(self) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker = None
            self._sos, self._zi = self._build_filter(self._fs_meas)
            self._n_samples   = 0
            self._total_drops = 0
            self._drop_lbl.setText('Drops: 0')
            self._drop_lbl.setStyleSheet('color: #aaa;')
            self._connect_btn.setText('Connect')
            self._status_lbl.setText('●  Disconnected')
            self._status_lbl.setStyleSheet('color: #888;')
        else:
            port = self._port_combo.currentText()
            if not port:
                return
            baud = int(self._baud_combo.currentText())
            sensitivity = FSR_OPTIONS[self._fsr_combo.currentText()]
            self._sensitivity = sensitivity
            self._start_worker(SerialWorker(port, baud, sensitivity))
            self._connect_btn.setText('Disconnect')
            self._status_lbl.setText(f'●  {port}')
            self._status_lbl.setStyleSheet('color: #4f4;')

    def _start_worker(self, worker: SerialWorker | DemoWorker) -> None:
        self.worker = worker
        self.worker.batch_ready.connect(self._on_batch)
        self.worker.start()
        if self.demo:
            self._status_lbl.setText('●  DEMO')
            self._status_lbl.setStyleSheet('color: #fa0;')

    # ── Data pipeline ─────────────────────────────────────────────────────────

    def _on_batch(self, batch: np.ndarray, t0: float, t1: float, drops: int) -> None:  # noqa: ARG002
        n = len(batch)

        if drops:
            self._total_drops += drops
            self._drop_lbl.setText(f'Drops: {self._total_drops}')
            self._drop_lbl.setStyleSheet('color: #f44;')

        # FS is fixed by the firmware timer — chunked serial reads make wall-clock
        # estimation unreliable (parsing time << sample period).

        filt, self._zi = sosfilt(self._sos, batch, axis=0, zi=self._zi)

        p = self._ring_ptr
        if p + n <= HISTORY:
            self._ring[p:p+n]      = batch
            self._ring_filt[p:p+n] = filt
        else:
            first = HISTORY - p
            self._ring[p:]      = batch[:first]; self._ring[:n-first]      = batch[first:]
            self._ring_filt[p:] = filt[:first];  self._ring_filt[:n-first] = filt[first:]

        self._ring_ptr   = (p + n) % HISTORY
        self._n_samples += n

        # Welch runs at DISPLAY_WELCH_HZ so the spectrum stays live. The control
        # update is gated separately in _on_fft_done on CTRL_SAMPLES of new data.
        if (self._n_samples >= WELCH_MIN_SAMPLES
                and self._n_samples - self._last_welch_n >= DISPLAY_WELCH_SAMPLES
                and not self._fft_running):
            self._last_welch_n = self._n_samples
            self._launch_welch()

    def _launch_welch(self) -> None:
        self._fft_running = True
        n = min(CTRL_SAMPLES, self._n_samples, HISTORY)
        # Most recent n samples, ending at the write pointer.
        start = (self._ring_ptr - n) % HISTORY
        if start + n <= HISTORY:
            snap = self._ring_filt[start:start + n].astype(np.float64)
        else:
            first = HISTORY - start
            snap = np.empty((n, CHANNEL_COUNT), dtype=np.float64)
            snap[:first] = self._ring_filt[start:]
            snap[first:] = self._ring_filt[:n - first]
        runnable = WelchRunnable(snap, self._fs_meas, self._on_fft_done)
        self._pool.start(runnable)

    @QtCore.pyqtSlot(object)
    def _on_fft_done(self, results: list[np.ndarray]) -> None:
        self._fft_running = False

        # `results` is the raw, fresh, non-overlapping window — the ONLY thing
        # the control loop may use. The displayed PSD is an EMA of it purely to
        # look as smooth as the old long-window average; feeding that back into
        # the loop would reintroduce exactly the lag that caused the limit cycle.
        if self._psd is None or self._psd[0].shape != results[0].shape:
            self._psd = [r.copy() for r in results]
        else:
            a = 1.0 - math.exp(-1.0 / (DISPLAY_WELCH_HZ * PSD_DISPLAY_TAU_S))
            for i, r in enumerate(results):
                self._psd[i] += a * (r - self._psd[i])

        ctrl = self._profile_dock.control_channels()
        mask = self._plot_mask
        df   = SAMPLE_RATE / SPEC_N   # 1.0 Hz per bin

        # Display runs every Welch; control only on a fresh non-overlapping
        # window, so it never reacts to data it has already acted on.
        ctrl_due = (self._n_samples - self._last_ctrl_n) >= CTRL_SAMPLES
        if ctrl_due:
            self._last_ctrl_n = self._n_samples

        # ── Measured Grms display ─────────────────────────────────────────
        if ctrl:
            grms_sq = sum(
                float(np.sum(results[CHANNEL_NAMES.index(ch)][mask])) * df
                for ch in ctrl if ch in CHANNEL_NAMES
            )
            grms = math.sqrt(max(grms_sq, 0.0))
            self._profile_dock.update_measured_grms(grms)
            if self._profile_dock.is_running:
                now = self._profile_dock.test_elapsed_s()
                self._grms_now_line.setPos(now)
                self._grms_now_line.setVisible(True)
                if ctrl_due:
                    # One point per independent window — a trend, not a smear.
                    self._grms_t.append(now)
                    self._grms_v.append(grms)
                    self._grms_meas_curve.setData(self._grms_t, self._grms_v)

        # ── Spectral error, in-spec status, and closed-loop correction ────
        if ctrl:
            ctrl_indices = [CHANNEL_NAMES.index(ch) for ch in ctrl
                            if ch in CHANNEL_NAMES]

            # Average PSD across control channels (mean, not RSS, for spectral shaping)
            meas_psd = np.mean(
                [results[i][mask] for i in ctrl_indices], axis=0
            )

            bp = self._profile_dock.breakpoints()
            if len(bp) < 2:
                return

            demand_psd = psd_interp_loglog(
                self._plot_freqs, bp, self._profile_dock.current_gain_db(),
            )

            # Control only inside the profile band — the drive synthesises nothing
            # outside it, so correcting there does nothing except pin the error
            # readout high and saturate the clamp.
            band = ((self._plot_freqs >= bp[0][0]) &
                    (self._plot_freqs <= bp[-1][0]))
            if not band.any():
                return

            # dB error: positive → we're below demand → need to drive harder
            err_db = 10.0 * np.log10(demand_psd) - 10.0 * np.log10(meas_psd)
            err_db[~band] = 0.0

            # In-spec status tracks the live display, not the control cadence —
            # it is meaningful even with the loop switched off.
            in_band_err = np.abs(err_db[band])
            self._profile_dock.update_spec_status(
                float(np.mean(in_band_err > TOL_ALARM_DB)),
                float(np.mean(in_band_err > TOL_ABORT_DB)),
            )

            if not (self._profile_dock.loop_enabled and ctrl_due):
                return

            # Ensure correction arrays have the right size (resets can resize _plot_freqs)
            if self._H_corr_db.shape != err_db.shape:
                self._H_corr_db = np.zeros_like(err_db)
            if self._H_base_db.shape != err_db.shape:
                self._H_base_db = np.zeros_like(err_db)

            # Split the error. The common-mode part is a pure level deficit; the
            # per-bin loop is blind to it (unit-RMS normalisation cancels any
            # uniform correction exactly), so it goes to the level servo. What
            # remains is zero-mean and is genuinely about spectral shape — which
            # is also the only part that survives the normalisation.
            common = float(np.mean(err_db[band]))
            self._profile_dock.nudge_level_db(
                float(np.clip(common, -CTRL_LEVEL_MAX_STEP_DB,
                              CTRL_LEVEL_MAX_STEP_DB)))
            # err_db stays intact for the readouts — reporting the zero-mean
            # residual would hide a pure level deficit entirely.
            err_shape = err_db - common
            err_shape[~band] = 0.0

            # Smooth error across frequency — one noisy bin must not swing its
            # own correction independently of its neighbours.
            err_smooth = gaussian_filter1d(err_shape, sigma=5.0)

            # Soft deadband: shrink toward zero rather than hard-gating, so the
            # correction stops random-walking on measurement noise once
            # converged, without chattering at the threshold.
            err_eff = np.sign(err_smooth) * np.maximum(
                np.abs(err_smooth) - CTRL_DEADBAND_DB, 0.0)

            # Integral update. The plant is memoryless in dB
            # (measured_dB = drive_dB + plant_dB), so error decays as
            # (1 - gain)^n — deadbeat at gain 1, stable below 2. Valid only
            # because each update now sees a fresh, non-overlapping window.
            max_corr = self._profile_dock.max_correction_db
            corr_hi  = max_corr
            corr_lo  = -min(max_corr, CTRL_MAX_CUT_DB)
            # Asymmetric response. Growing |correction| uses the user's gain and
            # the tight slew limit, because that is where stability is at stake.
            # Shrinking it is a return to neutral — the worst case is landing at
            # zero correction, i.e. driving the raw profile — so it runs at unity
            # gain and may unwind a full rail in a single update.
            raw     = self._profile_dock.loop_gain * err_eff
            toward  = (raw * self._H_corr_db) < 0.0
            raw     = np.where(toward, err_eff, raw)
            lim     = np.where(toward, CTRL_MAX_UNWIND_DB, CTRL_MAX_STEP_DB)
            step    = np.clip(raw, -lim, lim)
            # ...but never fling a bin through zero out the other side; land on it.
            over        = toward & (np.abs(step) > np.abs(self._H_corr_db))
            step[over]  = -self._H_corr_db[over]

            # Anti-windup by conditional integration: never accumulate further
            # into a rail we are already sitting on. Without this a bin the rig
            # cannot reach keeps integrating, so when conditions change it has to
            # unwind through tens of dB before the drive responds at all — which
            # is what stalls recovery and looks like the loop being stuck.
            at_hi = (self._H_corr_db >= corr_hi - 1e-9) & (step > 0.0)
            at_lo = (self._H_corr_db <= corr_lo + 1e-9) & (step < 0.0)
            step[at_hi | at_lo] = 0.0

            self._H_corr_db += step
            np.clip(self._H_corr_db, corr_lo, corr_hi, out=self._H_corr_db)
            self._H_corr_db[~band] = 0.0

            self._profile_dock.push_correction(self._plot_freqs, self._H_corr_db)
            hc = self._H_corr_db[band]
            self._profile_dock.update_loop_error(
                float(np.sqrt(np.mean(err_db[band] ** 2))),
                float(np.mean((hc >= corr_hi - 1e-6) | (hc <= corr_lo + 1e-6))),
            )

            # Correction trace on the right-hand dB axis: how hard each frequency
            # is being pushed relative to the profile. Once converged this is the
            # inverse of the rig's transfer function.
            self._drive_curve.setData(self._log_plot_freqs[band],
                                      self._H_corr_db[band])
            self._drive_curve.setVisible(True)
            span = max(10.0, float(np.max(np.abs(self._H_corr_db[band]))) * 1.15)
            self._corr_vb.setYRange(-span, span, padding=0)

    # ── Display ───────────────────────────────────────────────────────────────

    def _refresh_display(self) -> None:
        p = self._ring_ptr
        self._disp[:HISTORY - p] = self._ring[p:]
        self._disp[HISTORY - p:] = self._ring[:p]

        n  = self._plot_samples
        t  = self._t_axis[-n:]
        for idx, name in enumerate(CHANNEL_NAMES):
            if self._curves[name].isVisible():
                self._curves[name].setData(t, self._disp[-n:, idx])

        if self._psd is not None:
            mask = self._plot_mask
            lf   = self._log_plot_freqs
            logs = {}
            for idx, name in enumerate(CHANNEL_NAMES):
                if self._spec_curves[name].isVisible():
                    logs[name] = np.log10(np.maximum(self._psd[idx][mask], 1e-30))

            # Adaptive floor. Anchored to the profile so the tolerance band stays
            # legible once converged, but dropped to reveal the measured trace
            # when the rig is far off target (open loop, or outside its band).
            # Bounded span, so a bin on the correction clamp still cannot blow
            # the scale out; hysteresis, so it does not jitter every frame.
            bm = self._spec_band_mask
            if logs and bm is not None and bm.any():
                # In-band only: the roll-off outside the profile is not what the
                # test is judged on, and letting it set the scale would compress
                # the tolerance band for no reason.
                data_lo = min(float(np.percentile(v[bm], 1.0))
                              for v in logs.values())
                want_lo = min(self._spec_base_lo, data_lo - 0.3)
                want_lo = max(want_lo, self._spec_hi - PSD_VIEW_MAX_DECADES)
                if abs(want_lo - self._spec_floor_log) > 0.25:
                    self._spec_floor_log = want_lo
                    self._spec_plot.setYRange(want_lo, self._spec_hi, padding=0)

            floor = self._spec_floor_log
            for name, v in logs.items():
                self._spec_curves[name].setData(lf, np.maximum(v, floor))

    def closeEvent(self, a0: QtGui.QCloseEvent | None) -> None:
        if self.worker:
            self.worker.stop()
        self._profile_dock._on_audio_stop()
        self._pool.waitForDone(1000)
        super().closeEvent(a0)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description='IMU Visualizer')
    parser.add_argument('--demo', action='store_true',
                        help='Run with simulated data — no hardware required')
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    pg.setConfigOptions(antialias=False, useOpenGL=True, foreground='w', background='#1e1e1e')

    win = MainWindow(demo=args.demo)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
