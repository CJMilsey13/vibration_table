"""Host side of the wire protocol: the drop counter (ticket 08) and the
repeat-count tool for ticket 14."""

import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))
from count_repeats import analyse, parse_frames


def frame(seq, ax=0, ay=0, az=0):
    return b'\xAA\x55' + struct.pack('<H3h', seq & 0xFFFF, ax, ay, az)


class FakeSerial:
    """Delivers a byte string in chunks, then stops the worker."""

    def __init__(self, data, worker):
        self.data, self.worker = data, worker

    def set_buffer_size(self, **_):
        pass

    def read(self, n):
        chunk, self.data = self.data[:n], self.data[n:]
        if not self.data:
            self.worker._running = False
        return chunk

    def close(self):
        pass


def run_worker(iv, monkeypatch, data, infos=None):
    w = iv.SerialWorker('FAKE', 115200, 2048.0)
    monkeypatch.setattr(iv.serial, 'Serial', lambda *a, **k: FakeSerial(data, w))
    batches, failures = [], []
    w.batch_ready.connect(lambda b, d: batches.append((b, d)))
    w.failed.connect(failures.append)
    if infos is not None:
        w.info_ready.connect(infos.append)
    w.run()
    assert not failures
    return batches


def test_contiguous_stream_reports_no_drops(iv, monkeypatch):
    data = b''.join(frame(s, az=2048) for s in range(800))
    batches = run_worker(iv, monkeypatch, data)
    assert len(batches) == 10 and sum(d for _, d in batches) == 0
    assert np.allclose(batches[0][0][:, 2], 1.0)     # 2048 LSB = 1 g at ±16 g


def test_firmware_overrun_shows_up_as_drops(iv, monkeypatch):
    """With seq stamped at acquisition, a 1 s host stall against the 4096-deep
    ring leaves a gap of 8000 - 4096 samples. The host must count exactly that."""
    lost  = 8000 - 4096
    seqs  = list(range(400)) + list(range(400 + lost, 400 + lost + 400))
    batches = run_worker(iv, monkeypatch, b''.join(frame(s) for s in seqs))
    assert sum(d for _, d in batches) == lost


def test_drop_count_survives_sequence_wraparound(iv, monkeypatch):
    seqs = list(range(65300, 65536)) + list(range(0, 164))          # no loss across the wrap
    assert sum(d for _, d in run_worker(iv, monkeypatch, b''.join(frame(s) for s in seqs))) == 0
    seqs = list(range(65300, 65500)) + list(range(100, 300))        # 136 lost across the wrap
    assert sum(d for _, d in run_worker(iv, monkeypatch, b''.join(frame(s) for s in seqs))) == 136


def test_parser_resyncs_after_garbage(iv, monkeypatch):
    data = b'\x00\x01\x02' + b''.join(frame(s) for s in range(160))
    batches = run_worker(iv, monkeypatch, data)
    assert len(batches) == 2 and sum(d for _, d in batches) == 0


# ── Firmware status line ─────────────────────────────────────────────────────

BANNER = (b'# icm42688_streamer 2026-10-08 drdy-polled odr=8107.419 '
          b'reset=11,0D,30,40,62 now=01,0D,7E,80,3F\n')


def test_status_line_between_frames_is_reported_and_costs_no_frames(iv, monkeypatch):
    data = (BANNER + b''.join(frame(s, az=2048) for s in range(400)) +
            BANNER + b''.join(frame(s, az=2048) for s in range(400, 800)))
    infos = []
    batches = run_worker(iv, monkeypatch, data, infos)
    assert len(batches) == 10 and sum(d for _, d in batches) == 0
    assert np.allclose(np.concatenate([b for b, _ in batches])[:, 2], 1.0)
    assert infos == [BANNER.decode().strip()] * 2
    from stream import banner_rate
    assert banner_rate(infos[0]) == 8107.419


def test_status_line_split_across_reads_is_still_found(iv, monkeypatch):
    # FakeSerial hands out 4096-byte chunks; put the line across a boundary.
    lead = b''.join(frame(s) for s in range(407))            # 4070 bytes
    data = lead + BANNER + b''.join(frame(s) for s in range(407, 880))
    assert len(lead) < 4096 < len(lead) + len(BANNER)
    infos = []
    batches = run_worker(iv, monkeypatch, data, infos)
    assert sum(d for _, d in batches) == 0 and len(batches) == 11
    assert infos == [BANNER.decode().strip()]


def test_garbage_is_not_reported_as_a_status_line(iv, monkeypatch):
    infos = []
    data = b'\x00\x01 not a banner\n' + b''.join(frame(s) for s in range(160))
    run_worker(iv, monkeypatch, data, infos)
    assert infos == []


# ── tools/count_repeats.py ───────────────────────────────────────────────────

def test_count_repeats_finds_duplicated_samples(rng):
    n = 8000
    samples = rng.integers(-2000, 2000, size=(n, 3))
    dup_at = np.arange(100, n, 100)                  # one re-read every 100 polls = 80 Hz
    samples[dup_at] = samples[dup_at - 1]
    data = b'junk' + b''.join(frame(i, *map(int, s)) for i, s in enumerate(samples)) + b'\xAA'
    seqs, parsed = parse_frames(data)
    assert np.array_equal(parsed, samples)
    r = analyse(seqs, parsed)
    assert r['frames'] == n and r['dropped'] == 0
    assert r['repeats'] == len(dup_at)
    assert abs(r['repeat_rate_hz'] - 79.0) < 1.0
    assert r['gap_mean'] == 100.0 and r['gap_std'] == 0.0


def test_count_repeats_counts_sequence_gaps():
    seqs = np.array([65534, 65535, 0, 1, 5, 6], dtype=np.uint16)
    samples = np.arange(18, dtype=np.int16).reshape(6, 3)
    r = analyse(seqs, samples)
    assert r['dropped'] == 3 and r['repeats'] == 0
    assert analyse(seqs[:1], samples[:1])['frames'] == 1


def test_parse_frames_skips_the_status_line():
    data = BANNER + frame(7, 1, 2, 3) + BANNER + frame(8, 4, 5, 6)
    seqs, samples = parse_frames(data)
    assert list(seqs) == [7, 8] and samples.tolist() == [[1, 2, 3], [4, 5, 6]]
