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
  = 10 bytes per sample, at the SENSOR's rate (8 kHz nominal, ~8108 Hz actual)
  Between frames the firmware sends a '#' status line every 2 s carrying
  odr=<measured rate>; the stream is resampled to exactly 8000 Hz from it.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path
from typing import Optional

import threading

import numpy as np
import pyqtgraph as pg
from scipy.signal import welch as scipy_welch
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtGui, QtWidgets

from control import (
    CTRL_MAX_CUT_DB, LEVEL_MAX_DBFS_RANGE, SPEC_AVG_DEFAULT, SPEC_AVG_MAX,
    TOL_ABORT_DB, TOL_ALARM_DB,
    ResponseError, SpecAssessor, SpecStatus, SpectralController,
    band_grms, band_mask, breakpoint_error, gaussian_clip, psd_grms,
    psd_interp_loglog,
)
from sequence import SequenceRunner
from stream import (
    CTRL_WINDOW_S, DISPLAY_WELCH_HZ, HISTORY, PSD_FMAX, PSD_FMIN,
    SAMPLE_RATE, SPEC_N, WINDOW_TIME, MeasurementStream, Window, banner_rate,
)

try:
    import sounddevice as _sd
    HAS_SOUNDDEVICE = True
except ImportError:
    _sd = None          # type: ignore[assignment]
    HAS_SOUNDDEVICE = False


# ── Constants ─────────────────────────────────────────────────────────────────
# Measurement timing lives in stream.py, control-law tuning in control.py.

CHANNEL_DICT: dict[str, tuple[int, int, int]] = {
    'Accel X': (255, 200,   0),
    'Accel Y': (210,  80, 255),
    'Accel Z': (  0, 215, 215),
}
CHANNEL_COUNT = len(CHANNEL_DICT)
CHANNEL_NAMES = list(CHANNEL_DICT.keys())

PSD_DISPLAY_TAU_S     = 0.35   # EMA time constant, display only — never control

BATCH    = 80
# A block counts as clipped once this share of its samples hit the rail. One
# flattened peak in a few thousand is inaudible and 50 dB down; the indicator
# is for clipping that is actually distorting the drive.
CLIP_FRACTION = 5e-3
PSD_FLOOR_LOG = -12.0   # log10(g²/Hz) placeholder for "no data yet"
# Spectrum view window, in decades either side of the demand profile. Kept tight
# on purpose: the ±6 dB tolerance band is only 0.6 of a decade, so a very wide
# view compresses the one thing the operator is reading into a few pixels.
PSD_VIEW_DECADES_BELOW = -4.0
PSD_VIEW_DECADES_ABOVE =  1.0
PSD_VIEW_MAX_DECADES   =  8.0   # hard cap, so a clamped bin can't blow the scale

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
    batch_ready = QtCore.pyqtSignal(object, int)   # samples, dropped frames
    failed      = QtCore.pyqtSignal(str)           # port could not be opened / was lost
    info_ready  = QtCore.pyqtSignal(str)           # a '# icm42688_streamer …' status line

    BANNER = b'# icm42688_streamer'

    def __init__(self, port: str, baud: int, sensitivity: float) -> None:
        super().__init__()
        self.port        = port
        self.baud        = baud
        self.sensitivity = sensitivity
        self._running    = False

    def run(self) -> None:
        FRAME_BYTES = 2 + PAYLOAD_BYTES
        SYNC        = bytes([SYNC_A, SYNC_B])
        READ_CHUNK  = 4096

        buf      = np.empty((BATCH, CHANNEL_COUNT), dtype=np.float32)
        idx      = 0
        drops    = 0
        last_seq: Optional[int] = None
        scale    = 1.0 / self.sensitivity
        pending  = bytearray()
        text     = bytearray()      # bytes that were not part of a frame

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
                        text.extend(pending[:-1])
                        pending = pending[-1:]
                        break
                    if sync_pos > 0:
                        # Not a frame: the firmware's status line, or garbage.
                        text.extend(pending[:sync_pos])
                        pending = pending[sync_pos:]
                    if text:
                        self._scan_text(text)
                    if len(pending) < FRAME_BYTES:
                        break

                    seq, ax, ay, az = struct.unpack('<H3h', pending[2:FRAME_BYTES])
                    pending = pending[FRAME_BYTES:]

                    if last_seq is not None:
                        expected = (last_seq + 1) & 0xFFFF
                        if seq != expected:
                            drops += (seq - expected) & 0xFFFF
                    last_seq = seq

                    buf[idx, 0] = ax * scale
                    buf[idx, 1] = ay * scale
                    buf[idx, 2] = az * scale
                    idx += 1

                    if idx == BATCH:
                        self.batch_ready.emit(buf.copy(), drops)
                        drops = 0
                        idx   = 0

            ser.close()
        except serial.SerialException as exc:
            self.failed.emit(str(exc))

    def _scan_text(self, text: bytearray) -> None:
        """Emit any complete status line held in `text`, and consume it."""
        while True:
            end = text.find(b'\n')
            if end < 0:
                break
            line = bytes(text[:end])
            del text[:end + 1]
            at = line.find(self.BANNER)
            if at >= 0:
                self.info_ready.emit(line[at:].decode('ascii', 'replace').strip())
        if len(text) > 512:             # never a line — do not let it grow
            del text[:-512]

    def stop(self) -> None:
        self._running = False
        self.wait(2000)


# ── Demo worker ───────────────────────────────────────────────────────────────

class DemoWorker(QtCore.QThread):
    """Synthetic 8 kHz data: 50 Hz + 120 Hz + 1 kHz peaks."""
    batch_ready = QtCore.pyqtSignal(object, int)

    def run(self) -> None:
        self._running = True
        t   = 0.0
        dt  = 1.0 / SAMPLE_RATE
        buf = np.empty((BATCH, CHANNEL_COUNT), dtype=np.float32)
        idx = 0

        while self._running:
            buf[idx, 0] = (0.5 * math.sin(2*math.pi*50*t)
                         + 0.1 * math.sin(2*math.pi*1000*t))
            buf[idx, 1] =  0.3 * math.sin(2*math.pi*120*t + 1.0)
            buf[idx, 2] =  0.2 * math.sin(2*math.pi*75*t)
            t   += dt
            idx += 1
            if idx == BATCH:
                self.batch_ready.emit(buf.copy(), 0)
                idx = 0
                self.msleep(int(1000 * BATCH / SAMPLE_RATE))

    def stop(self) -> None:
        self._running = False
        self.wait(2000)


# ── Welch PSD runnable ────────────────────────────────────────────────────────

class WelchRunnable(QtCore.QRunnable):
    """50%-overlapping Hann-windowed Welch PSD, run in thread pool."""

    def __init__(self, window: Window, fs: float, callback) -> None:
        super().__init__()
        self.setAutoDelete(True)
        self._window   = window
        self._fs       = fs
        self._callback = callback

    def run(self) -> None:
        results: list[np.ndarray] = []
        for ch in range(CHANNEL_COUNT):
            _, psd = scipy_welch(
                self._window.samples[:, ch],
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
            # The window travels with its PSD, so the receiver can check it
            # is still valid for a control update when the result arrives.
            QtCore.Q_ARG(object, (results, self._window)),
        )


# ── Audio output worker ───────────────────────────────────────────────────────

class AudioOutputWorker(QtCore.QThread):
    """
    Streams spectrally-shaped Gaussian noise to the system audio DAC.

    The PSD profile is IFFT-shaped each block: for each frequency bin f,
    amplitude = sqrt(PSD(f) * df), random phase.  The block is normalised to
    unit RMS before output_gain is applied, so output_gain alone controls the
    DAC level independent of the profile shape or absolute Grms.

    Clip detection fires clip_detected for EVERY block in which more than
    CLIP_FRACTION of the samples exceed ±1.0 after output_gain scaling, so the
    indicator stays lit for as long as the clipping lasts.
    """

    clip_detected = QtCore.pyqtSignal()
    failed        = QtCore.pyqtSignal(str)   # output stream could not be opened / died

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

    def render(self, frames: int, rng: np.random.Generator) -> tuple[np.ndarray, bool]:
        """One output block at the current settings, and whether it clipped."""
        with self._lock:
            bp      = self._breakpoints
            gain    = self._gain_db
            og      = self._output_gain
            corr_f  = self._corr_freqs
            corr_db = self._corr_db

        sig = _generate_shaped_block(bp, gain, frames, self._fs, rng, corr_f, corr_db)
        sig *= og

        over    = np.abs(sig) > 1.0
        clipped = bool(np.mean(over) > CLIP_FRACTION)
        if over.any():
            np.clip(sig, -1.0, 1.0, out=sig)
        return sig, clipped

    def run(self) -> None:
        if not HAS_SOUNDDEVICE:
            self.failed.emit('sounddevice is not installed')
            return

        rng = np.random.default_rng()   # per-thread RNG, never shared

        def callback(outdata: np.ndarray, frames: int, _time, _status) -> None:
            sig, clipped = self.render(frames, rng)
            if clipped:
                self.clip_detected.emit()

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
            if self._running:           # not a failure if we were asked to stop
                self.failed.emit(str(exc))

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

    Returns a UNIT-RMS block. Only the profile's shape survives: gain_db scales
    every bin alike and is divided straight back out by the normalisation, so
    it cannot change the level. Sequence gain steps reach the DAC through the
    output gain (SpectralController.output_dbfs), never through here.

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
        # ...but the WHOLE band, to its edges. Each drive bin is fs/n wide
        # (~11 Hz), so a bin is included if any of its width lies in the band:
        # that is half a bin of slack either side. Stopping at the last bin
        # centre inside the band leaves up to half a bin at the top with no
        # drive at all, and the loop then winds those few bins toward the
        # boost rail chasing energy that is not being made.
        half = 0.5 * fs / n
        mask = (freqs >= f_lo - half) & (freqs <= f_hi + half)
    else:
        f_lo = f_hi = 0.0
        mask = np.zeros(len(freqs), dtype=bool)
    if mask.any():
        df       = fs / n
        psd_vals = psd_interp_loglog(freqs[mask], breakpoints, gain_db)
        if corr_freqs is not None and corr_db is not None and len(corr_freqs) >= 2:
            # Interpolate correction onto this block's grid (log-linear).
            # Taper to 0 dB outside the control band — holding a saturated edge
            # value here multiplies out-of-band power by up to 10^(max_corr/10).
            # A bin centred just outside the band (see `half` above) is there
            # to drive the band's edge, so it takes the edge's correction.
            c = np.interp(
                np.log10(np.clip(freqs[mask], f_lo, f_hi)),
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
        rows = [list(r) for r in self._rows]
        rows[index.row()][index.column()] = v
        rows.sort(key=lambda r: r[0])
        # Reject the edit outright rather than hold a table the PSD maths
        # cannot use — a duplicate frequency is a zero-width segment.
        if breakpoint_error([(r[0], r[1]) for r in rows]) is not None:
            return False
        self._rows = rows
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
    profile_changed  = QtCore.pyqtSignal()             # breakpoint table edited
    sequence_changed = QtCore.pyqtSignal()             # sequence table edited
    step_started     = QtCore.pyqtSignal(int, float)   # step_idx, gain_db
    test_started     = QtCore.pyqtSignal()              # fired once on Start
    test_stopped     = QtCore.pyqtSignal()
    reset_requested  = QtCore.pyqtSignal()             # ↺ Reset pressed
    save_response_requested = QtCore.pyqtSignal()
    load_response_requested = QtCore.pyqtSignal()
    audio_started           = QtCore.pyqtSignal()
    audio_stopped           = QtCore.pyqtSignal()
    audio_failed            = QtCore.pyqtSignal(str)
    # The drive changed in a way the loop did not command: the slider, a
    # sequence step, Drive On. Data measured before it must not be controlled on.
    drive_changed           = QtCore.pyqtSignal()

    def __init__(self, controller: SpectralController, parent=None) -> None:
        super().__init__('Test Profile', parent)
        self.setFeatures(
            QtWidgets.QDockWidget.DockWidgetMovable |
            QtWidgets.QDockWidget.DockWidgetFloatable
        )
        # The controller owns the drive level; this dock only seeds it from the
        # slider and relays it to the audio worker.
        self._ctl       = controller
        self._bp_model  = BreakpointModel()
        self._seq_model = SequenceModel()
        self._runner    = SequenceRunner(self._seq_model.steps())
        self._audio_worker: Optional[AudioOutputWorker] = None

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._tick)

        self._build_ui()
        self._bp_model.layoutChanged.connect(self._on_profile_changed)
        self._bp_model.dataChanged.connect(self._on_profile_changed)
        self._seq_model.layoutChanged.connect(self._on_sequence_changed)
        self._seq_model.dataChanged.connect(self._on_sequence_changed)

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
            f'Share of in-band bins inside ±{TOL_ALARM_DB:.0f} dB (alarm) and '
            f'±{TOL_ABORT_DB:.0f} dB (abort) of the demand profile,\n'
            f'judged on a running average of {CTRL_WINDOW_S:.0f} s control windows. '
            'A single window scatters\n'
            'by ~3 dB per bin on its own, so no verdict is given until the '
            'average is full.\n'
            'The number of windows is set on the Settings tab.'
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

        # Shown only while the level servo is on its ceiling and still short.
        self._limit_lbl = QtWidgets.QLabel('')
        self._limit_lbl.setWordWrap(True)
        self._limit_lbl.setStyleSheet(
            'font-family: Consolas; font-weight: bold; color: #f44; '
            'border: 1px solid #f44; padding: 3px;')
        self._limit_lbl.setToolTip(
            'The drive is at the highest level the PC is allowed to send\n'
            '(Settings → Max drive level) and the rig is still below demand.\n'
            'Sequence steps cannot raise it further from here.\n'
            '\n'
            'Turn the AMPLIFIER up by at least the amount shown, or lower the\n'
            'demand: a lower profile level, or a narrower band.\n'
            'Raising Max drive level instead makes the drive clip, and\n'
            'clipping distortion then sets the level at the rig\'s resonances.')
        self._limit_lbl.setVisible(False)
        cl_lay.addWidget(self._limit_lbl)

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
            'Starting drive level for the 0 dB profile. A sequence step adds\n'
            'its own gain on top. With the loop enabled this is a starting\n'
            'point, not a fixed setting — the level servo trims from here to\n'
            'match the measured Grms to demand. The value to the right is the\n'
            'level actually going to the DAC; it turns orange when it is being\n'
            'held down by Max drive level (Settings tab). Moving the slider\n'
            're-seeds the servo, as do Drive On and Reset.'
        )
        self._audio_slider.valueChanged.connect(self._on_audio_level_changed)
        self._ctl.reseed_level(float(self._audio_slider.value()))
        self._audio_level_lbl = QtWidgets.QLabel()
        self._audio_level_lbl.setStyleSheet('font-family: Consolas; color: #aaa;')
        self._audio_level_lbl.setFixedWidth(55)
        self._apply_level()
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

    def _on_sequence_changed(self) -> None:
        self._update_total_label()
        if self._runner.set_steps(self._seq_model.steps()):
            self._complete_test()     # the step being run was deleted
        elif self._runner.in_progress:
            self._apply_level()       # the current step's gain may have changed
            self._update_step_display()
            self.drive_changed.emit()
        self.sequence_changed.emit()

    # ── Channel helpers ───────────────────────────────────────────────────────

    def control_channels(self) -> list[str]:
        return [n for n, cb in self._ch_checks.items() if cb.isChecked()]

    # ── Test runner ───────────────────────────────────────────────────────────

    def _start_test(self) -> None:
        if not self._runner.start():
            return
        self._start_btn.setEnabled(False)
        self._pause_btn.setEnabled(True)
        self._stop_btn.setEnabled(True)
        self.test_started.emit()                     # → MainWindow resets correction
        self._enter_step()
        if HAS_SOUNDDEVICE and self._audio_worker is None:
            self._on_audio_start()                   # auto-start drive at step-0 gain
        self._timer.start()

    def _pause_test(self) -> None:
        """Hold: the drive keeps its current level and shape, the loop freezes."""
        if not self._runner.in_progress:
            return
        if self._runner.is_paused:
            self._runner.resume()
            self._timer.start()
            self._pause_btn.setText('⏸  Pause')
            self._step_lbl.setStyleSheet('color: #4f4; font-family: Consolas;')
        else:
            self._runner.pause()
            self._timer.stop()
            self._pause_btn.setText('▶  Resume')
            self._step_lbl.setStyleSheet('color: #fa0; font-family: Consolas;')

    def _stop_test(self) -> None:
        self._end_test('Stopped', '#888', 0)

    def _complete_test(self) -> None:
        self._end_test('Complete ✓', '#4f4', 100)

    def _end_test(self, text: str, colour: str, progress: int) -> None:
        # Stop and completion both stop the drive. Outside a test the demand
        # is the 0 dB profile, so leaving the drive (and the loop) running
        # would raise the level the moment the test ended.
        self._timer.stop()
        self._runner.stop()
        self._start_btn.setEnabled(True)
        self._pause_btn.setEnabled(False)
        self._pause_btn.setText('⏸  Pause')
        self._stop_btn.setEnabled(False)
        self._step_lbl.setText(text)
        self._step_lbl.setStyleSheet(f'color: {colour}; font-family: Consolas;')
        self._progress.setValue(progress)
        self._on_audio_stop()
        self.test_stopped.emit()

    def _enter_step(self) -> None:
        self.step_started.emit(self._runner.step_index, self._runner.gain_db)
        self._push_audio_profile()
        self._apply_level()          # the step's gain is a level change at the DAC
        self._update_step_display()
        self.drive_changed.emit()

    def _tick(self) -> None:
        changed = self._runner.tick(1.0)
        if self._runner.state == SequenceRunner.COMPLETE:
            self._complete_test()
        elif changed:
            self._enter_step()
        else:
            self._update_step_display()

    def _update_step_display(self) -> None:
        steps = self._runner.steps
        idx   = self._runner.step_index
        if not self._runner.in_progress or idx >= len(steps):
            return
        dur, gain = steps[idx]
        remaining = max(0, int(dur - self._runner.step_elapsed_s))
        m, s = divmod(remaining, 60)
        target_g = psd_grms(self._bp_model.breakpoints(), gain)
        self._step_lbl.setText(
            f'Step {idx+1}/{len(steps)}  ·  {gain:+.1f} dB  ·  '
            f'{target_g:.3f} g  ·  {m}:{s:02d} left'
        )
        if not self._runner.is_paused:
            self._step_lbl.setStyleSheet('color: #4f4; font-family: Consolas;')
        self._progress.setValue(
            min(100, int(100 * self._runner.step_elapsed_s / dur)))
        self._grms_target_lbl.setText(f'Target: {target_g:.3f} g')
        self._grms_target_lbl.setStyleSheet('color: #fff; font-family: Consolas;')

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
        self._ctl.reseed_level(float(db_val))
        self._apply_level()
        self.drive_changed.emit()

    def _apply_level(self) -> None:
        """Show the level going to the DAC and hand it to the audio worker.

        Call after anything that moves it: the slider, the level servo, or a
        sequence step — whose gain is applied here, as output level, because
        the synthesised block is unit-RMS whatever gain the profile carries.
        Every caller except the servo must also emit drive_changed.
        """
        gain   = self.current_gain_db()
        out_db = self._ctl.output_dbfs(gain)
        self._audio_level_lbl.setText(f'{out_db:+.1f} dB')
        # Orange when Max drive level is holding it below what was asked for.
        self._audio_level_lbl.setStyleSheet(
            'font-family: Consolas; color: %s;'
            % ('#fa0' if self._ctl.output_limited(gain) else '#aaa'))
        if self._audio_worker is not None:
            self._audio_worker.set_output_gain(10.0 ** (out_db / 20.0))

    @property
    def drive_running(self) -> bool:
        return self._audio_worker is not None

    def _on_audio_start(self) -> None:
        if self._audio_worker is not None:
            return
        # Always start from the operator's level, never from wherever the
        # servo was left by an earlier run.
        self._ctl.reseed_level(float(self._audio_slider.value()))
        w = AudioOutputWorker()
        w.set_profile(self._bp_model.breakpoints(), self.current_gain_db())
        dev_idx = self._audio_dev_combo.currentData()
        if dev_idx is not None:
            w.set_device(int(dev_idx))
        fs_text = self._audio_rate_combo.currentText()
        w.set_fs(int(fs_text))
        w.clip_detected.connect(self._on_clip_detected)
        w.failed.connect(self._on_audio_failed)
        self._audio_worker = w
        self._apply_level()
        w.start()
        self._audio_start_btn.setEnabled(False)
        self._audio_stop_btn.setEnabled(True)
        self._audio_clip_lbl.setText('')
        # Hand the worker whatever correction is already learned/loaded —
        # otherwise a saved response is ignored until the next Welch cycle,
        # and never applied at all when the loop is disabled.
        self.audio_started.emit()
        self.drive_changed.emit()

    def _on_audio_stop(self) -> None:
        if self._audio_worker is None:
            return
        self._audio_worker.stop()
        self._audio_worker = None
        self._audio_start_btn.setEnabled(True)
        self._audio_stop_btn.setEnabled(False)
        self._audio_clip_lbl.setText('')
        self.audio_stopped.emit()

    def _on_audio_failed(self, message: str) -> None:
        if self.sender() is not self._audio_worker:
            return                    # a worker we have already let go of
        # The stream is dead: say so, and do not leave a test running against
        # a drive that is not there.
        self._on_audio_stop()
        if self._runner.in_progress:
            self._stop_test()
        self.audio_failed.emit(message)

    def _on_clip_detected(self) -> None:
        # Fires for every clipping block, and each one restarts the timer, so
        # the label stays up for as long as the clipping does.
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

    def update_loop_idle(self, reason: str) -> None:
        self._ctrl_err_lbl.setText(f'Error RMS: —  ({reason})')
        self._ctrl_err_lbl.setStyleSheet('font-family: Consolas; color: #aaa;')
        self.update_drive_limit(None)

    def update_drive_limit(self, shortfall_db: Optional[float]) -> None:
        """Say that the drive is at its limit and how far short, or clear it."""
        if shortfall_db is None:
            self._limit_lbl.setVisible(False)
            return
        self._limit_lbl.setText(
            f'DRIVE AT LIMIT ({self._ctl.max_output_dbfs:+.0f} dBFS) — rig is '
            f'{shortfall_db:.1f} dB short. Turn the amplifier up.')
        self._limit_lbl.setVisible(True)

    def update_spec_status(self, status: Optional[SpecStatus]) -> None:
        """In-spec readout: share of controlled bins inside each tolerance."""
        if status is None:
            txt, colour = '—', '#aaa'
        else:
            verdict = status.verdict
            colour = {'ABORT': '#f44', 'ALARM': '#fa0',
                      'IN SPEC': '#4f4'}.get(verdict, '#aaa')
            if not status.settled:
                # Too few averages for a verdict — the scatter alone would fail it.
                verdict += f' {status.n_windows}/{status.n_avg}'
            txt = (f'{verdict}   ±{TOL_ALARM_DB:.0f}dB {(1-status.alarm_frac)*100:.0f}%   '
                   f'±{TOL_ABORT_DB:.0f}dB {(1-status.abort_frac)*100:.0f}%')
        self._spec_status_lbl.setText(txt)
        border = colour if colour != '#aaa' else '#555'
        self._spec_status_lbl.setStyleSheet(
            f'font-family: Consolas; font-weight: bold; color: {colour}; '
            f'border: 1px solid {border}; padding: 3px;')

    def _on_ctrl_reset(self) -> None:
        # Reset means "start over from what the operator set": the saved
        # response for shape, the slider for level.
        self._ctl.reseed_level(float(self._audio_slider.value()))
        self.reset_requested.emit()   # MainWindow resets the correction
        self._apply_level()

    def _push_audio_profile(self) -> None:
        if self._audio_worker is not None:
            self._audio_worker.set_profile(
                self._bp_model.breakpoints(), self.current_gain_db()
            )

    def current_gain_db(self) -> float:
        return self._runner.gain_db

    @property
    def is_running(self) -> bool:
        return self._runner.in_progress

    @property
    def is_paused(self) -> bool:
        return self._runner.is_paused

    def test_elapsed_s(self) -> float:
        """Elapsed across the whole sequence — not the per-step clock."""
        return self._runner.elapsed_s

    def demand_grms_trace(self) -> tuple[np.ndarray, np.ndarray]:
        """Target Grms staircase for the entire sequence, computed up front."""
        return self._runner.plan(self._bp_model.breakpoints())

    def update_measured_grms(self, grms: float) -> None:
        self._grms_meas_lbl.setText(f'Meas: {grms:.3f} g')
        self._grms_meas_lbl.setStyleSheet(
            'color: #4f4; font-family: Consolas;' if grms > 0 else 'color: #aaa; font-family: Consolas;'
        )


# ── Main window ───────────────────────────────────────────────────────────────

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, demo: bool = False) -> None:
        super().__init__()
        self.demo   = demo
        self.worker: Optional[SerialWorker | DemoWorker] = None

        self._sensitivity  = FSR_OPTIONS[FSR_DEFAULT]
        self._total_drops  = 0

        # Ring buffers, bandpass state and both Welch gates.
        self._stream = MeasurementStream(CHANNEL_COUNT)
        self._plot_samples: int       = int(2.0 * SAMPLE_RATE)  # initial display: 2 s
        self._t_axis:      np.ndarray = np.linspace(-WINDOW_TIME, 0.0, HISTORY, dtype=np.float32)

        freqs                = np.fft.rfftfreq(SPEC_N, 1.0 / SAMPLE_RATE)
        self._plot_mask      = (freqs >= PSD_FMIN) & (freqs <= PSD_FMAX)
        self._plot_freqs     = freqs[self._plot_mask]
        self._log_plot_freqs = np.log10(self._plot_freqs)

        # Level servo, shape loop and the saved speaker response. The correction
        # spectrum it holds is dB power on _plot_freqs, 0 = no correction.
        self._ctl      = SpectralController(self._plot_freqs)
        # Running average behind the IN SPEC / ALARM / ABORT readout.
        self._assessor = SpecAssessor(self._plot_freqs, SPEC_AVG_DEFAULT)

        self._spec_floor_log: float = PSD_FLOOR_LOG
        self._spec_base_lo:   float = PSD_FLOOR_LOG
        self._spec_hi:        float = 0.0
        self._spec_band_mask: Optional[np.ndarray] = None

        self._psd:         Optional[list[np.ndarray]] = None   # EMA, display only
        self._fft_running: bool = False

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

        # Instantiate the status bar up-front so later messages don't reflow the layout.
        self.statusBar().showMessage('Ready')

        # Start from the rig's previously learned response, if we have one.
        self._load_response(announce=False)

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

        # Two views: the live plots, and settings that are not touched mid-run.
        self._tabs = QtWidgets.QTabWidget()
        self._tabs.addTab(self._build_plot_panel(), 'Live')
        self._tabs.addTab(self._build_settings_panel(), 'Settings')
        root.addWidget(self._tabs, stretch=1)

        # Test profile dock — right side
        self._profile_dock = TestProfileDock(self._ctl, self)
        self._profile_dock.setMinimumWidth(290)
        self._profile_dock.setMaximumWidth(360)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self._profile_dock)

        self._profile_dock.profile_changed.connect(self._recompute_target)
        self._profile_dock.profile_changed.connect(self._reset_correction)
        self._profile_dock.profile_changed.connect(self._rebuild_grms_demand)
        # A sequence edit changes the plan and maybe the current gain, but not
        # the rig — so it does not reset what the loop has learned.
        self._profile_dock.sequence_changed.connect(self._recompute_target)
        self._profile_dock.sequence_changed.connect(self._rebuild_grms_demand)
        self._profile_dock.step_started.connect(self._on_step_started)
        self._profile_dock.test_started.connect(self._reset_correction)
        self._profile_dock.test_started.connect(self._on_test_started)
        self._profile_dock.test_stopped.connect(self._on_test_stopped)
        self._profile_dock.reset_requested.connect(self._reset_correction)
        self._profile_dock.save_response_requested.connect(self._save_response)
        self._profile_dock.load_response_requested.connect(self._load_response)
        self._profile_dock.audio_started.connect(self._on_drive_started)
        self._profile_dock.audio_stopped.connect(self._on_drive_stopped)
        self._profile_dock.audio_failed.connect(self._on_drive_failed)
        self._profile_dock.drive_changed.connect(self._stream.restart_control_window)

    def _build_settings_panel(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setContentsMargins(12, 12, 12, 12)

        grp  = QtWidgets.QGroupBox('In-spec status')
        form = QtWidgets.QFormLayout(grp)

        self._spec_avg_spin = QtWidgets.QSpinBox()
        self._spec_avg_spin.setRange(1, SPEC_AVG_MAX)
        self._spec_avg_spin.setValue(self._assessor.n_avg)
        self._spec_avg_spin.setSuffix(' windows')
        self._spec_avg_spin.setFixedWidth(120)
        self._spec_avg_lbl = QtWidgets.QLabel()
        self._spec_avg_lbl.setStyleSheet('color: #aaa; font-family: Consolas;')
        avg_row = QtWidgets.QHBoxLayout()
        avg_row.addWidget(self._spec_avg_spin)
        avg_row.addWidget(self._spec_avg_lbl)
        avg_row.addStretch()
        form.addRow('Averaging:', avg_row)

        note = QtWidgets.QLabel(
            f'The IN SPEC / ALARM / ABORT readout is judged on a running average '
            f'of the last N control windows ({CTRL_WINDOW_S:.0f} s each, '
            f'non-overlapping). One window alone scatters by about 2.9 dB per '
            f'bin, which fails ±{TOL_ALARM_DB:.0f} dB on estimator noise however '
            f'good the rig is.\n\n'
            f'More windows give a steadier verdict but a slower one: the readout '
            f'shows AVERAGING until N windows are in, and a real excursion takes '
            f'up to N windows to show fully.\n\n'
            f'This affects the readout only. The control loop always acts on '
            f'each single fresh window.')
        note.setWordWrap(True)
        note.setStyleSheet('color: #aaa;')
        form.addRow(note)

        self._spec_avg_spin.valueChanged.connect(self._on_spec_avg_changed)
        self._on_spec_avg_changed(self._spec_avg_spin.value())

        layout.addWidget(grp)

        drv  = QtWidgets.QGroupBox('Drive')
        dfrm = QtWidgets.QFormLayout(drv)
        self._max_drive_spin = QtWidgets.QDoubleSpinBox()
        self._max_drive_spin.setRange(*LEVEL_MAX_DBFS_RANGE)
        self._max_drive_spin.setDecimals(1)
        self._max_drive_spin.setSingleStep(1.0)
        self._max_drive_spin.setValue(self._ctl.max_output_dbfs)
        self._max_drive_spin.setSuffix(' dBFS')
        self._max_drive_spin.setFixedWidth(120)
        self._max_drive_lbl = QtWidgets.QLabel()
        self._max_drive_lbl.setStyleSheet('color: #aaa; font-family: Consolas;')
        drive_row = QtWidgets.QHBoxLayout()
        drive_row.addWidget(self._max_drive_spin)
        drive_row.addWidget(self._max_drive_lbl)
        drive_row.addStretch()
        dfrm.addRow('Max drive level:', drive_row)
        drive_note = QtWidgets.QLabel(
            'The highest RMS level the PC will send to the amplifier. The level '
            'servo stops here and shows DRIVE AT LIMIT, with how far short the '
            'rig is.\n\n'
            'The drive is Gaussian noise, so its peaks are several times its RMS. '
            'Above about −12 dBFS those peaks hit the DAC rail. The distortion '
            'that makes is broadband: it puts energy at the rig\'s resonances no '
            'matter how far the loop has cut the drive there, so the resonance '
            'stays hot and the rest of the band is starved. At 0 dBFS a third of '
            'the samples are clipped and the distortion is only 10 dB down.\n\n'
            'If the rig cannot reach demand at this level, turn the amplifier '
            'up. Raise this setting only if the amplifier has nothing left, and '
            'expect the spectrum to go out of shape when you do.')
        drive_note.setWordWrap(True)
        drive_note.setStyleSheet('color: #aaa;')
        dfrm.addRow(drive_note)
        self._max_drive_spin.valueChanged.connect(self._on_max_drive_changed)
        self._on_max_drive_changed(self._max_drive_spin.value())
        layout.addWidget(drv)

        layout.addStretch()
        return widget

    def _on_max_drive_changed(self, dbfs: float) -> None:
        self._ctl.max_output_dbfs = float(dbfs)
        frac, sdr = gaussian_clip(dbfs)
        self._max_drive_lbl.setText(
            f'clips {frac * 100:.3g} % of samples   '
            f'(distortion {sdr:.0f} dB below the drive)')
        # Not there yet while the settings tab is first being built.
        dock = getattr(self, '_profile_dock', None)
        if dock is not None:
            dock._apply_level()
            self._stream.restart_control_window()   # the drive level may have jumped

    def _on_spec_avg_changed(self, n: int) -> None:
        self._assessor.n_avg = n
        self._spec_avg_lbl.setText(
            f'= {n * CTRL_WINDOW_S:.0f} s of data   '
            f'(per-bin scatter ≈ {2.9 / math.sqrt(n):.1f} dB)')

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
            f'The spectrum always uses the most recent {CTRL_WINDOW_S:.0f} s.'
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
        band     = band_mask(self._plot_freqs, bp)
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
        self._ctl.reset()
        self._apply_correction()
        self._stream.restart_control_window()   # the correction just jumped
        self._assessor.reset()
        self._profile_dock.update_spec_status(None)
        self._profile_dock._ctrl_err_lbl.setText('Error RMS: —')
        self._profile_dock._ctrl_err_lbl.setStyleSheet('font-family: Consolas; color: #aaa;')
        self._profile_dock.update_drive_limit(None)
        self._drive_curve.setVisible(False)

    def _apply_correction(self) -> None:
        """Hand the audio worker the correction the controller currently holds."""
        corr = self._ctl.corr_db
        if np.any(corr):
            self._profile_dock.push_correction(self._plot_freqs, corr)
        else:
            self._profile_dock.clear_correction()

    @QtCore.pyqtSlot()
    def _on_drive_started(self) -> None:
        # A new worker starts with no correction — give it what is already
        # learned/loaded, or a saved response would be ignored until the next
        # control update, and never applied at all with the loop disabled.
        self._apply_correction()
        self._assessor.reset()

    @QtCore.pyqtSlot()
    def _on_drive_stopped(self) -> None:
        self._assessor.reset()
        self._profile_dock.update_spec_status(None)
        self._profile_dock.update_drive_limit(None)

    @QtCore.pyqtSlot(str)
    def _on_drive_failed(self, message: str) -> None:
        self.statusBar().showMessage(f'Audio output failed — drive is OFF: {message}')

    # ── Speaker response persistence ──────────────────────────────────────────

    @QtCore.pyqtSlot()
    def _save_response(self) -> None:
        if not np.any(self._ctl.corr_db):
            QtWidgets.QMessageBox.information(
                self, 'Save speaker response',
                'No correction to save yet — run the loop until the error '
                'converges, then save.',
            )
            return
        try:
            self._ctl.save_response(RESPONSE_FILE)
        except OSError as exc:
            QtWidgets.QMessageBox.warning(
                self, 'Save speaker response', f'Could not write file:\n{exc}')
            return
        self.statusBar().showMessage(
            f'Speaker response saved to {RESPONSE_FILE.name} '
            f'(peak {np.max(np.abs(self._ctl.base_db)):.1f} dB)', 5000)

    @QtCore.pyqtSlot()
    def _load_response(self, announce: bool = True) -> None:
        if not RESPONSE_FILE.exists():
            if announce:
                QtWidgets.QMessageBox.information(
                    self, 'Load speaker response',
                    f'No saved response found at:\n{RESPONSE_FILE}')
            return
        # The file is checked against the clamp the loop is running with now
        # (boost up to Max corr, cut down to CTRL_MAX_CUT_DB).
        self._ctl.max_boost_db = self._profile_dock.max_correction_db
        try:
            self._ctl.load_response(RESPONSE_FILE)
        except ResponseError as exc:
            if announce:
                QtWidgets.QMessageBox.warning(
                    self, 'Load speaker response',
                    f'{RESPONSE_FILE.name} was not loaded.\n\n{exc}')
            else:
                # Startup: no dialog, but leave it up until something replaces it.
                self.statusBar().showMessage(
                    f'{RESPONSE_FILE.name} NOT loaded — {exc}')
            return
        self._apply_correction()
        self._stream.restart_control_window()   # the correction just jumped
        self.statusBar().showMessage(
            f'Speaker response loaded from {RESPONSE_FILE.name} '
            f'(peak {np.max(np.abs(self._ctl.base_db)):.1f} dB)', 8000)

    # ── Connection management ─────────────────────────────────────────────────

    def _refresh_ports(self) -> None:
        self._port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self._port_combo.addItem(p.device)

    def _toggle_connection(self) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker = None
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
        # New session: clear the stream — data, filter state and BOTH Welch
        # gates together — and everything that was averaged from the old one.
        # Assume the nominal rate again until this device says otherwise.
        self._stream.set_input_rate(SAMPLE_RATE)
        self._stream.reset()
        self._psd = None
        self._assessor.reset()
        self._fs_lbl.setText(f'FS: {SAMPLE_RATE:.0f} Hz (configured)')
        self._fs_lbl.setToolTip('')
        self.worker = worker
        self.worker.batch_ready.connect(self._on_batch)
        if isinstance(worker, SerialWorker):
            worker.failed.connect(self._on_serial_failed)
            worker.info_ready.connect(self._on_serial_info)
        self.worker.start()
        if self.demo:
            self._status_lbl.setText('●  DEMO')
            self._status_lbl.setStyleSheet('color: #fa0;')

    @QtCore.pyqtSlot(str)
    def _on_serial_info(self, line: str) -> None:
        """The firmware's status line: adopt the sensor rate it measured."""
        if self.sender() is not self.worker:
            return
        rate = banner_rate(line)
        if rate is None:
            return
        if self._stream.set_input_rate(rate):
            # The stream restarted on the right time base; what was averaged
            # from the old one is on the wrong frequency axis.
            self._psd = None
            self._assessor.reset()
            self._profile_dock.update_spec_status(None)
        self._fs_lbl.setText(
            f'FS: {SAMPLE_RATE:.0f} Hz (sensor {rate:.1f} Hz, resampled)')
        self._fs_lbl.setToolTip(line)

    @QtCore.pyqtSlot(str)
    def _on_serial_failed(self, message: str) -> None:
        worker = self.sender()
        if worker is not self.worker or worker is None:
            return                    # a worker we have already let go of
        self.worker = None
        worker.wait(2000)
        self._connect_btn.setText('Connect')
        self._status_lbl.setText('●  Connection failed')
        self._status_lbl.setStyleSheet('color: #f44;')
        self.statusBar().showMessage(f'Serial error: {message}')

    # ── Data pipeline ─────────────────────────────────────────────────────────

    def _on_batch(self, batch: np.ndarray, drops: int) -> None:
        if drops:
            self._total_drops += drops
            self._drop_lbl.setText(f'Drops: {self._total_drops}')
            self._drop_lbl.setStyleSheet('color: #f44;')

        # FS is fixed by the firmware timer — chunked serial reads make wall-clock
        # estimation unreliable (parsing time << sample period).
        self._stream.push(batch)

        # Welch runs at DISPLAY_WELCH_HZ so the spectrum stays live. Whether a
        # window may also drive a control update is decided by the stream: only
        # on CTRL_SAMPLES of new data, all of it gathered since the drive last
        # changed.
        if not self._fft_running:
            window = self._stream.take_window()
            if window is not None:
                self._fft_running = True
                self._pool.start(
                    WelchRunnable(window, SAMPLE_RATE, self._on_welch_done))

    @QtCore.pyqtSlot(object)
    def _on_welch_done(self, payload: tuple) -> None:
        results, window = payload
        self._fft_running = False
        if not self._stream.is_current(window):
            return      # computed from a session that has since been reset
        self._on_fft_done(results, self._stream.is_control_window(window))

    def _on_fft_done(self, results: list[np.ndarray], is_control: bool = True) -> None:
        # `results` is the raw, fresh window — the ONLY thing the control loop
        # may use, and only when is_control says it shares no samples with the
        # last one. The displayed PSD is an EMA of it purely to look smooth;
        # feeding that back into the loop would reintroduce exactly the lag
        # that caused the limit cycle.
        if self._psd is None or self._psd[0].shape != results[0].shape:
            self._psd = [r.copy() for r in results]
        else:
            a = 1.0 - math.exp(-1.0 / (DISPLAY_WELCH_HZ * PSD_DISPLAY_TAU_S))
            for i, r in enumerate(results):
                self._psd[i] += a * (r - self._psd[i])

        dock = self._profile_dock
        ctrl_indices = [CHANNEL_NAMES.index(ch) for ch in dock.control_channels()
                        if ch in CHANNEL_NAMES]
        bp = dock.breakpoints()
        if not ctrl_indices or len(bp) < 2:
            return
        gain_db = dock.current_gain_db()

        # ONE estimate for everything below — the mean PSD across the control
        # channels (mean, not RSS, for spectral shaping). The Grms readout, the
        # in-spec status and the loop all read it, so they cannot disagree.
        meas_psd = np.mean(
            [results[i][self._plot_mask] for i in ctrl_indices], axis=0
        )

        # ── Measured Grms display ─────────────────────────────────────────
        grms = band_grms(self._plot_freqs, meas_psd, bp)
        dock.update_measured_grms(grms)
        if dock.is_running:
            now = dock.test_elapsed_s()
            self._grms_now_line.setPos(now)
            self._grms_now_line.setVisible(True)
            if is_control:
                # One point per independent window — a trend, not a smear.
                self._grms_t.append(now)
                self._grms_v.append(grms)
                self._grms_meas_curve.setData(self._grms_t, self._grms_v)

        # Everything below runs once per fresh non-overlapping window, so it
        # never reacts to data it has already acted on.
        if not is_control:
            return

        # ── In-spec status ────────────────────────────────────────────────
        # Reports what the rig is doing, not what the controller is doing, so
        # it is computed with the loop off too.
        self._assessor.add(meas_psd, bp, gain_db)
        dock.update_spec_status(self._assessor.status(bp))

        # ── Closed-loop correction ────────────────────────────────────────
        if not dock.loop_enabled:
            return
        self._ctl.loop_gain    = dock.loop_gain
        self._ctl.max_boost_db = dock.max_correction_db
        # With no drive playing the measurement is ambient noise, and a paused
        # test is a hold — neither may be integrated.
        out = self._ctl.update(
            meas_psd, bp, gain_db,
            drive_active=dock.drive_running and not dock.is_paused,
        )
        if not out.acted:
            dock.update_loop_idle('paused' if dock.is_paused else 'drive off')
            return

        dock._apply_level()
        dock.push_correction(self._plot_freqs, out.corr_db)
        dock.update_loop_error(out.err_rms_db, out.sat_frac)
        dock.update_drive_limit(out.shortfall_db if out.at_limit else None)

        # Correction trace on the right-hand dB axis: how hard each frequency
        # is being pushed relative to the profile. Once converged this is the
        # inverse of the rig's transfer function.
        band = out.band
        self._drive_curve.setData(self._log_plot_freqs[band], out.corr_db[band])
        self._drive_curve.setVisible(True)
        span = max(10.0, float(np.max(np.abs(out.corr_db[band]))) * 1.15)
        self._corr_vb.setYRange(-span, span, padding=0)

    # ── Display ───────────────────────────────────────────────────────────────

    def _refresh_display(self) -> None:
        n    = self._plot_samples
        t    = self._t_axis[-n:]
        disp = self._stream.recent_raw(n)
        for idx, name in enumerate(CHANNEL_NAMES):
            if self._curves[name].isVisible():
                self._curves[name].setData(t, disp[:, idx])

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
