"""
Evidence for ticket 14: is the firmware re-reading the same sensor sample?

The firmware polls the IMU on the MCU's 125 µs timer; the IMU produces samples
on its own 8 kHz clock. If the two drift, a poll sometimes lands twice inside
one sensor period and returns the same sample again (and, drifting the other
way, skips one — which cannot be seen from the host).

Run with the rig STILL and the drive OFF:

    python tools/count_repeats.py COM5            # 10 s capture
    python tools/count_repeats.py COM5 --seconds 30

Reading the result: sensor noise alone makes all three axes repeat exactly now
and then, so some repeats are expected. Clock slip shows as repeats arriving at
a STEADY rate — the "spacing" line gives the mean and spread of the gaps
between them. A rate of R repeats/s means the MCU is polling R Hz faster than
the sensor is sampling.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import numpy as np

SYNC        = b'\xAA\x55'
FRAME_BYTES = 10
SAMPLE_RATE = 8000.0


def parse_frames(data: bytes) -> tuple[np.ndarray, np.ndarray]:
    """→ (seq uint16 array, samples int16 array of shape (n, 3)). Resyncs on
    the sync word, so a capture may start or end mid-frame."""
    seqs: list[int] = []
    rows: list[tuple[int, int, int]] = []
    i, n = 0, len(data)
    while i + FRAME_BYTES <= n:
        if data[i:i + 2] != SYNC:
            j = data.find(SYNC, i + 1)
            if j < 0:
                break
            i = j
            continue
        seq, ax, ay, az = struct.unpack_from('<H3h', data, i + 2)
        seqs.append(seq)
        rows.append((ax, ay, az))
        i += FRAME_BYTES
    return (np.asarray(seqs, dtype=np.uint16),
            np.asarray(rows, dtype=np.int16).reshape(-1, 3))


def analyse(seqs: np.ndarray, samples: np.ndarray) -> dict:
    """Count frames identical to the one before on all three axes, and gaps in
    the sequence number (samples the firmware dropped on overrun)."""
    if len(samples) < 2:
        return {'frames': len(samples), 'repeats': 0, 'dropped': 0,
                'repeat_rate_hz': 0.0, 'gap_mean': float('nan'),
                'gap_std': float('nan')}
    same    = np.all(samples[1:] == samples[:-1], axis=1)
    at      = np.flatnonzero(same) + 1
    step    = np.diff(seqs.astype(np.int64)) & 0xFFFF
    dropped = int(np.sum(step - 1))
    gaps    = np.diff(at)
    return {
        'frames':         len(samples),
        'repeats':        int(same.sum()),
        'dropped':        dropped,
        'repeat_rate_hz': float(same.sum()) / (len(samples) / SAMPLE_RATE),
        'gap_mean':       float(gaps.mean()) if len(gaps) else float('nan'),
        'gap_std':        float(gaps.std())  if len(gaps) else float('nan'),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('port')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seconds', type=float, default=10.0)
    args = ap.parse_args()

    import serial   # only needed for a live capture

    buf = bytearray()
    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.set_buffer_size(rx_size=131072)
        ser.reset_input_buffer()
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            buf.extend(ser.read(4096))

    r = analyse(*parse_frames(bytes(buf)))
    secs = r['frames'] / SAMPLE_RATE
    print(f"frames    : {r['frames']}  ({secs:.2f} s at 8 kHz, "
          f"{args.seconds:.2f} s wall clock)")
    print(f"dropped   : {r['dropped']}  (sequence gaps)")
    print(f"repeats   : {r['repeats']}  = {r['repeat_rate_hz']:.2f} per second "
          f"= {r['repeat_rate_hz'] / SAMPLE_RATE * 1e6:.0f} ppm clock offset if steady")
    print(f"spacing   : mean {r['gap_mean']:.1f} frames, std {r['gap_std']:.1f}  "
          '(std much smaller than mean → steady → clock slip)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
