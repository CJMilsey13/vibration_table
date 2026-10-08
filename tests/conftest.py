import os
import sys
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import pytest


@pytest.fixture
def rng():
    # Fixed so a failure reproduces. Set TEST_SEED to try other noise.
    return np.random.default_rng(int(os.environ.get('TEST_SEED', '20261007')))


@pytest.fixture(scope='session')
def app():
    from PyQt5 import QtWidgets
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def iv(monkeypatch, tmp_path, app):
    """The visualizer module with no real audio device and no real response file."""
    import imu_visualizer as iv

    monkeypatch.setattr(iv, 'RESPONSE_FILE', tmp_path / 'speaker_response.json')
    # "Drive On" creates the worker but never opens a sound device.
    monkeypatch.setattr(iv.AudioOutputWorker, 'start', lambda self: None)
    monkeypatch.setattr(iv.AudioOutputWorker, 'stop', lambda self: None)
    monkeypatch.setattr(iv, 'HAS_SOUNDDEVICE', True)
    return iv


@pytest.fixture
def win(iv):
    w = iv.MainWindow(demo=False)
    w._display_timer.stop()
    yield w
    w.close()
