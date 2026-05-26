"""
IMU Visualizer — Accel X/Y/Z time-series + Welch PSD
=====================================================
Usage:
  python imu_visualizer.py              # connect to real hardware
  python imu_visualizer.py --demo       # run with simulated data (no hardware)

Requirements:  pip install pyqtgraph PyQt5 pyserial numpy scipy
Hardware:      ICM-42688-P via SPI → Pimoroni Pico Plus 2 (RP2350) → USB CDC

Wire protocol (firmware → host):
  [0xAA][0x55] | seq uint16 LE | ax int16 LE | ay int16 LE | az int16 LE
  = 10 bytes per sample at 8 kHz nominal
"""

from __future__ import annotations  # https://cjmilsey.atlassian.net/wiki/spaces/PP/pages/121438209/Forward+References+from+__future__+import+annotations

import argparse
import math
import struct
import sys
from typing import Optional

import numpy as np
import pyqtgraph as pg
from scipy.signal import butter, sosfilt, sosfilt_zi, welch as scipy_welch
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtGui, QtWidgets


# ── Constants ─────────────────────────────────────────────────────────────────

CHANNEL_DICT: dict[str, tuple[int, int, int]] = {
    'Accel X': (255, 200,   0),
    'Accel Y': (210,  80, 255),
    'Accel Z': (  0, 215, 215),
}
CHANNEL_COUNT = len(CHANNEL_DICT)
CHANNEL_NAMES = list(CHANNEL_DICT.keys())

SAMPLE_RATE = 1000.0          # nominal — updated live from timestamps
WINDOW_TIME = 2.0             # seconds of history kept / fed to Welch
HISTORY     = int(SAMPLE_RATE * WINDOW_TIME)   # 2 000 samples

SPEC_N   = 1000    # Welch segment = 1 s → 1 Hz resolution; ~3 averaged segments
BATCH    = 10      # samples per signal emission → 100 signals/s at 1 kHz
PSD_FMIN = 5.0     # Hz — PSD + filter lower bound
PSD_FMAX = 500.0   # Hz — Nyquist at 1 kHz

# Frame: 0xAA 0x55 | seq uint16 LE | ax int16 LE | ay int16 LE | az int16 LE
SYNC_A, SYNC_B = 0xAA, 0x55
PAYLOAD_BYTES  = 8    # 2 (seq) + 6 (3×int16)

# ICM-42688-P sensitivity in LSB/g for each FSR.
# Select the entry that matches ICM_ACCEL_CONFIG in firmware/main.c.
FSR_OPTIONS: dict[str, float] = {
    '±16 g': 2048.0,
    '±8 g':  4096.0,
    '±4 g':  8192.0,
    '±2 g': 16384.0,
}
FSR_DEFAULT = '±16 g'


# ── Custom log-scale axis with plain-number tick labels ───────────────────────

class LogHzAxis(pg.AxisItem):
    """
    Log-scale x-axis whose tick labels show plain Hz values (5, 10, 50, 1000 …)
    instead of pyqtgraph's default '10^x' notation.
    Curves must receive log10(Hz) as x data.
    """
    def tickStrings(self, values, scale, spacing):  # noqa: ARG002
        return [f'{10**v:.4g}' for v in values]


# ── Serial worker ─────────────────────────────────────────────────────────────

class SerialWorker(QtCore.QThread):
    """
    Reads ICM-42688-P binary frames from USB CDC.
    Emits (data_g float32, t0, t1, batch_drops) after BATCH samples.
    Changing FSR requires disconnect + reconnect (sensitivity is fixed at construction).
    """
    batch_ready = QtCore.pyqtSignal(object, float, float, int)

    def __init__(self, port: str, baud: int, sensitivity: float) -> None:
        super().__init__()
        self.port        = port
        self.baud        = baud
        self.sensitivity = sensitivity
        self._running    = False

    def run(self) -> None:
        import time as _time

        FRAME_BYTES = 2 + PAYLOAD_BYTES      # 10 bytes total
        SYNC        = bytes([SYNC_A, SYNC_B])
        READ_CHUNK  = 4096                   # bytes per read() call

        buf      = np.empty((BATCH, CHANNEL_COUNT), dtype=np.float32)
        idx      = 0
        t0       = 0.0
        drops    = 0
        last_seq: Optional[int] = None
        scale    = 1.0 / self.sensitivity
        pending  = bytearray()

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
            ser.set_buffer_size(rx_size=131072)   # 128 KB OS buffer
            self._running = True

            while self._running:
                # One large read instead of one per frame — far fewer syscalls
                incoming = ser.read(READ_CHUNK)
                if incoming:
                    pending.extend(incoming)

                # Parse every complete frame available in the buffer
                while len(pending) >= FRAME_BYTES:
                    # Find next sync header
                    sync_pos = pending.find(SYNC)
                    if sync_pos < 0:
                        # No sync in buffer; keep last byte in case it's 0xAA
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
    """Synthetic 8 kHz data: 50 Hz + 120 Hz + 1 kHz peaks visible across the full PSD range."""
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


# ── Welch PSD runnable (thread-pool) ─────────────────────────────────────────

class WelchRunnable(QtCore.QRunnable):
    """
    Runs scipy.signal.welch on a (HISTORY, N_CH) snapshot in a worker thread.
    Uses 50% overlapping SPEC_N-length Hann segments — ~3 averages at HISTORY=16k, SPEC_N=8k.
    """

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


# ── Main window ───────────────────────────────────────────────────────────────
# Top-level application window.
#
# Responsibilities:
#   - Owns and manages the background worker thread (SerialWorker or DemoWorker)
#   - Maintains ring buffers for raw and bandpass-filtered IMU samples
#   - Runs Welch PSD computation off the main thread via QThreadPool
#   - Drives a 30 Hz display timer that redraws time-series and PSD plots
#   - Handles serial port connection/disconnection and live FS estimation
#
# Key attributes:
#   self.worker        — active background thread, or None if disconnected
#   self._ring         — raw sample ring buffer        (HISTORY × N_CH)
#   self._ring_filt    — filtered sample ring buffer   (HISTORY × N_CH)
#   self._ring_ptr     — current write position in the ring buffers
#   self._fs_meas      — live-measured sample rate (updated each batch)
#   self._sensitivity  — ICM-42688-P LSB/g for the active FSR (set at connect time)
#   self._total_drops  — cumulative sequence-number gaps since last connect
#   self._psd          — most recent Welch PSD result, or None
# ─────────────────────────────────────────────────────────────────────────────
class MainWindow(QtWidgets.QMainWindow):

    _fft_done = QtCore.pyqtSignal(object)

    def __init__(self, demo: bool = False) -> None:
        super().__init__()
        self.demo   = demo
        self.worker: Optional[SerialWorker | DemoWorker] = None

        self._sensitivity  = FSR_OPTIONS[FSR_DEFAULT]
        self._total_drops  = 0

        # Ring buffers
        self._ring:      np.ndarray = np.zeros((HISTORY, CHANNEL_COUNT), dtype=np.float32)
        self._ring_filt: np.ndarray = np.zeros((HISTORY, CHANNEL_COUNT), dtype=np.float32)
        self._disp:      np.ndarray = np.zeros((HISTORY, CHANNEL_COUNT), dtype=np.float32)
        self._snap:      np.ndarray = np.zeros((HISTORY, CHANNEL_COUNT), dtype=np.float64)
        self._ring_ptr:  int        = 0
        self._n_samples: int        = 0

        # Measured FS
        self._fs_meas:    float       = SAMPLE_RATE
        self._fs_history: list[float] = []

        # Bandpass filter (PSD_FMIN – ~98% Nyquist) + initial freq arrays
        self._sos, self._zi = self._build_filter(SAMPLE_RATE)
        self._update_freq_arrays(SAMPLE_RATE)

        # PSD state
        self._psd:         Optional[list[np.ndarray]] = None
        self._fft_running: bool = False

        pool = QtCore.QThreadPool.globalInstance()
        assert pool is not None
        self._pool: QtCore.QThreadPool = pool

        self._build_ui()
        self._fs_lbl.setText(f'FS: {SAMPLE_RATE:.0f} Hz (configured)')
        self._fft_done.connect(self._on_fft_done)

        self._display_timer = QtCore.QTimer(self)
        self._display_timer.setInterval(33)
        self._display_timer.timeout.connect(self._refresh_display)
        self._display_timer.start()

        if demo:
            self._start_worker(DemoWorker())

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.setWindowTitle('IMU Visualizer — ICM-42688-P')
        self.resize(1400, 900)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setSpacing(4)
        root.setContentsMargins(6, 6, 6, 6)
        root.addLayout(self._build_toolbar())
        root.addWidget(self._build_plot_panel(), stretch=1)

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

        for w in (
            QtWidgets.QLabel('Port:'), self._port_combo, refresh_btn,
            QtWidgets.QLabel('Baud:'), self._baud_combo,
            QtWidgets.QLabel('FSR:'),  self._fsr_combo,
            self._connect_btn, self._status_lbl, self._fs_lbl, self._drop_lbl,
        ):
            bar.addWidget(w)
        bar.addStretch()
        return bar

    def _build_plot_panel(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        cb_box  = QtWidgets.QGroupBox('Visible Traces')
        cb_grid = QtWidgets.QGridLayout(cb_box)
        cb_grid.setVerticalSpacing(2)
        cb_grid.setHorizontalSpacing(10)

        # Time-series
        self._plot = pg.PlotWidget()
        self._plot.setLabel('left', 'Acceleration (g)')
        self._plot.setLabel('bottom', 'Samples')
        self._plot.showGrid(x=True, y=True, alpha=0.2)
        self._plot.setDownsampling(mode='peak')
        self._plot.setClipToView(True)
        self._plot.addLegend(offset=(8, 8))

        # PSD — custom log-Hz x-axis, linear y-axis (dB values passed directly)
        self._spec_plot = pg.PlotWidget(
            axisItems={'bottom': LogHzAxis(orientation='bottom')}
        )
        self._spec_plot.setLogMode(x=True, y=False)
        self._spec_plot.setLabel('left', 'PSD  (dB  re g²/Hz)')
        self._spec_plot.setLabel('bottom', 'Frequency  (Hz)')
        self._spec_plot.showGrid(x=True, y=True, alpha=0.2)
        self._spec_plot.addLegend(offset=(8, 8))
        self._spec_plot.getAxis('left').enableAutoSIPrefix(False)
        self._spec_plot.setXRange(
            math.log10(PSD_FMIN), math.log10(PSD_FMAX), padding=0
        )

        self._curves:      dict[str, pg.PlotDataItem] = {}
        self._spec_curves: dict[str, pg.PlotDataItem] = {}

        for idx, (name, color) in enumerate(CHANNEL_DICT.items()):
            self._curves[name] = self._plot.plot(
                np.zeros(HISTORY), name=name,
                pen=pg.mkPen(color=color, width=1.5),
            )
            self._curves[name].setDownsampling(ds=True, auto=True, method='peak')
            self._curves[name].setClipToView(True)

            self._spec_curves[name] = self._spec_plot.plot(
                self._log_plot_freqs,
                np.full(len(self._log_plot_freqs), -120.0),
                name=name, pen=pg.mkPen(color=color, width=1.5),
            )

            cb = QtWidgets.QCheckBox(name)
            cb.setChecked(True)
            cb.setStyleSheet(f'color: rgb{color};')
            cb.stateChanged.connect(
                lambda state, n=name: self._set_trace_visible(n, bool(state))
            )
            cb_grid.addWidget(cb, 0, idx)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        splitter.addWidget(self._plot)
        splitter.addWidget(self._spec_plot)
        splitter.setSizes([450, 350])

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
        lp_freq = min(PSD_FMAX - 5.0, fs * 0.49)   # stay clear of Nyquist
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

    def _on_batch(self, batch: np.ndarray, t0: float, t1: float, drops: int) -> None:
        n = len(batch)

        if drops:
            self._total_drops += drops
            self._drop_lbl.setText(f'Drops: {self._total_drops}')
            self._drop_lbl.setStyleSheet('color: #f44;')

        # Live FS estimation — median of last 20 batch rates
        if t1 > t0:
            self._fs_history.append((n - 1) / (t1 - t0))
            if len(self._fs_history) > 20:
                self._fs_history.pop(0)
            if len(self._fs_history) >= 5:
                self._update_fs(float(np.median(self._fs_history)))

        # Bandpass filter (stateful — no edge transients)
        filt, self._zi = sosfilt(self._sos, batch, axis=0, zi=self._zi)

        # Write into ring buffers
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

        if self._n_samples >= HISTORY and not self._fft_running:
            self._launch_welch()

    def _launch_welch(self) -> None:
        self._fft_running = True
        p = self._ring_ptr
        self._snap[:HISTORY - p] = self._ring_filt[p:]
        self._snap[HISTORY - p:] = self._ring_filt[:p]
        runnable = WelchRunnable(self._snap.copy(), self._fs_meas, self._on_fft_done)
        self._pool.start(runnable)

    @QtCore.pyqtSlot(object)
    def _on_fft_done(self, results: list[np.ndarray]) -> None:
        self._psd         = results
        self._fft_running = False

    # ── Display ───────────────────────────────────────────────────────────────

    def _refresh_display(self) -> None:
        p = self._ring_ptr
        self._disp[:HISTORY - p] = self._ring[p:]
        self._disp[HISTORY - p:] = self._ring[:p]

        for idx, name in enumerate(CHANNEL_NAMES):
            if self._curves[name].isVisible():
                self._curves[name].setData(self._disp[:, idx])

        if self._psd is not None:
            mask = self._plot_mask
            lf   = self._log_plot_freqs
            for idx, name in enumerate(CHANNEL_NAMES):
                if self._spec_curves[name].isVisible():
                    db = 10.0 * np.log10(self._psd[idx][mask])
                    self._spec_curves[name].setData(lf, db)

    def closeEvent(self, a0: QtGui.QCloseEvent | None) -> None:
        if self.worker:
            self.worker.stop()
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
