"""
Hardware checks for the IMU streamer. Read-only: nothing here drives the shaker.
Run with the rig STILL and the drive OFF.

    python tools/hw_check.py info  COM7
        Which firmware is running, and the sensor's filter registers at reset
        and as configured.

    python tools/hw_check.py rate  COM7 --seconds 60
        Sample rate against the PC clock, sequence gaps, and samples that are
        an exact repeat of the one before (tickets 8 and 14).

    python tools/hw_check.py stall COM7 --stall 3
        Stop reading for a few seconds, then count what the firmware reports
        as dropped (ticket 8). Every sample the sensor produced must be either
        received or counted as a drop: sequence numbers must still line up
        with the clock after the stall.

    python tools/hw_check.py noise COM7 --seconds 30
        At-rest noise density by frequency band (ticket 15). The sensor's own
        noise is white, so any roll-off across the band is the sensor's
        internal filtering.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from count_repeats import SAMPLE_RATE, analyse, parse_frames, parse_frames_at

LSB_PER_G = 2048.0          # ±16 g
BANDS_HZ  = [(100, 300), (300, 600), (600, 1000), (1000, 1400),
             (1400, 2000), (2000, 3000), (3000, 3900)]


def capture(port: str, seconds: float, baud: int = 115200,
            stall_at: float | None = None, stall_s: float = 0.0):
    """→ (bytes, [(t, n_bytes_so_far), ...]). Optionally stops reading for
    stall_s seconds once stall_at seconds have been captured."""
    import serial

    buf, marks = bytearray(), []
    with serial.Serial(port, baud, timeout=0.05) as ser:
        ser.reset_input_buffer()
        t0 = time.perf_counter()
        stalled = stall_at is None
        while True:
            now = time.perf_counter() - t0
            if now >= seconds:
                break
            if not stalled and now >= stall_at:
                time.sleep(stall_s)
                stalled = True
                marks.append((time.perf_counter() - t0, -1))     # marks the stall
                continue
            chunk = ser.read(4096)
            if chunk:
                buf.extend(chunk)
                marks.append((time.perf_counter() - t0, len(buf)))
    return bytes(buf), marks


def seq_span(seqs: np.ndarray) -> int:
    """Samples the firmware numbered between the first and last frame."""
    return int(np.sum(np.diff(seqs.astype(np.int64)) & 0xFFFF))


def find_banner(data: bytes) -> str | None:
    """The firmware's '# icm42688_streamer …' line, if this capture holds one."""
    i = data.find(b'# icm42688_streamer')
    if i < 0:
        return None
    j = data.find(b'\n', i)
    return data[i:j if j >= 0 else i + 120].decode('ascii', 'replace')


def cmd_info(args) -> dict:
    data, _ = capture(args.port, min(args.seconds, 5.0), args.baud)
    banner = find_banner(data)
    print(banner or 'no banner in 5 s — firmware older than 2026-10-08')
    if banner and 'reset=' in banner:
        names = ('GYRO_ACCEL_CONFIG0', 'ACCEL_CONFIG1', 'ACCEL_CONFIG_STATIC2',
                 'ACCEL_CONFIG_STATIC3', 'ACCEL_CONFIG_STATIC4')
        reset = banner.split('reset=')[1].split()[0].split(',')
        now   = banner.split('now=')[1].split()[0].split(',')
        for n, r, w in zip(names, reset, now):
            print(f'  {n:<22} reset 0x{r}   now 0x{w}')
        delt = lambda v: int(v, 16) >> 1
        print(f'  accel AAF DELT         reset {delt(reset[2])}   now {delt(now[2])}')
        print(f'  accel UI filter BW code reset {int(reset[0], 16) >> 4}   now {int(now[0], 16) >> 4}')
    return {'banner': banner}


def cmd_rate(args) -> dict:
    data, marks = capture(args.port, args.seconds, args.baud)
    seqs, samples = parse_frames(data)
    r = analyse(seqs, samples)
    # Rate from a straight-line fit of bytes against arrival time over every
    # chunk after the first second — far steadier than first-to-last, which
    # carries the full USB chunk jitter and anything buffered at open.
    t = np.array([m[0] for m in marks]); n = np.array([m[1] for m in marks], dtype=np.float64)
    keep = t > t[0] + 1.0
    wall = float(t[-1] - t[0])
    banner = find_banner(data)
    text_bytes_per_s = (len(banner) + 1) / 2.0 if banner else 0.0   # one line per 2 s
    fps  = (float(np.polyfit(t[keep], n[keep], 1)[0]) - text_bytes_per_s) / 10.0
    r.update(wall_s=wall, frames_per_s=fps, first_seq=int(seqs[0]))
    print(f"firmware       : {banner or 'no banner (older than 2026-10-08)'}")
    print(f"capture        : {wall:.2f} s wall clock, {r['frames']} frames")
    print(f"frames/s       : {fps:.2f}  ({(fps / SAMPLE_RATE - 1) * 1e6:+.0f} ppm vs 8000, by the PC clock)")
    print(f"first seq      : {r['first_seq']}")
    print(f"dropped        : {r['dropped']}  (sequence gaps)")
    print(f"exact repeats  : {r['repeats']}  = {r['repeat_rate_hz']:.2f} per second")
    print(f"repeat spacing : mean {r['gap_mean']:.1f} frames, std {r['gap_std']:.1f}")
    # Chance level: how often a sample equals the one TWO before it. Clock slip
    # re-reads only the immediately preceding sample, so it cannot raise this.
    if len(samples) > 2:
        lag2 = int(np.all(samples[2:] == samples[:-2], axis=1).sum())
        r['lag2_repeats'] = lag2
        print(f"lag-2 repeats  : {lag2}  = {lag2 / (len(samples) / SAMPLE_RATE):.2f} per second "
              '(chance level, for comparison)')
    return r


def cmd_stall(args) -> dict:
    total = args.before + args.stall + args.after
    data, marks = capture(args.port, total, args.baud,
                          stall_at=args.before, stall_s=args.stall)
    seqs, samples, ends = parse_frames_at(data)
    r = analyse(seqs, samples)
    step = np.diff(seqs.astype(np.int64)) & 0xFFFF
    unwrapped = np.concatenate([[0], np.cumsum(step)])        # sequence number, no wrap

    # For every chunk read: when it arrived, and the sequence number of the
    # last frame in it. Steady streaming puts these on a straight line.
    t_stall = next(t for t, n in marks if n < 0)              # when reading resumed
    t   = np.array([m[0] for m in marks if m[1] >= 0])
    idx = np.searchsorted(ends, [m[1] for m in marks if m[1] >= 0], side='right') - 1
    ok  = idx >= 0
    t, u = t[ok], unwrapped[idx[ok]].astype(np.float64)
    pre  = (t > 0.5) & (t < args.before - 0.1)
    post = t > t_stall + 1.5                                  # backlog has drained by then
    # One slope, two intercepts: u = a + b·t + c·[after the stall]. If the
    # firmware numbered every sample it lost, the two lines are the same line
    # and c = 0. Samples lost WITHOUT being numbered put the second line low.
    A = np.column_stack([np.ones(pre.sum() + post.sum()),
                         np.concatenate([t[pre], t[post]]),
                         np.concatenate([np.zeros(pre.sum()), np.ones(post.sum())])])
    (a0, rate, c), *_ = np.linalg.lstsq(A, np.concatenate([u[pre], u[post]]), rcond=None)
    resid = np.concatenate([u[pre], u[post]]) - A @ np.array([a0, rate, c])

    r.update(reported=r['dropped'], unreported=float(-c), rate=float(rate))
    print(f"firmware       : {find_banner(data) or 'no banner (older than 2026-10-08)'}")
    print(f"stall          : {args.stall:.1f} s without reading, {args.before:.0f} s before, {args.after:.0f} s after")
    print(f"frames received: {r['frames']}")
    print(f"reported drops : {r['dropped']}  in {int(np.sum(step > 1))} gap(s), largest {int(step.max()) - 1}")
    print(f"sample rate    : {rate:.1f} per second (fit to both sides of the stall)")
    print(f"unreported loss: {-c:+.0f} samples  (fit scatter ±{resid.std():.0f}; "
          '0 means every lost sample was numbered)')
    return r


def cmd_noise(args) -> dict:
    from scipy.signal import welch

    data, _ = capture(args.port, args.seconds, args.baud)
    seqs, samples = parse_frames(data)
    g = samples.astype(np.float64) / LSB_PER_G
    out = {'frames': len(g)}
    print(f"capture        : {len(g)} frames, {analyse(seqs, samples)['dropped']} dropped")
    print(f"rms            : " + '  '.join(
        f"{ax} {np.std(g[:, i]) * 1e3:.2f} mg" for i, ax in enumerate('XYZ')))
    print('band (Hz)        ' + ''.join(f'{ax:>22}' for ax in 'XYZ'))
    f, p = welch(g, fs=SAMPLE_RATE, window='hann', nperseg=8000, noverlap=4000, axis=0)
    ref = None
    for lo, hi in BANDS_HZ:
        m    = (f >= lo) & (f < hi)
        dens = np.sqrt(np.median(p[m], axis=0)) * 1e6           # µg/√Hz, robust to tones
        if ref is None:
            ref = dens
        out[f'{lo}-{hi}'] = dens.tolist()
        print(f'{lo:5d}-{hi:<5d}     ' + ''.join(
            f'{d:9.1f} µg/√Hz {20 * np.log10(d / r):+5.1f} dB' for d, r in zip(dens, ref)))
    if args.save:
        np.savez(args.save, f=f, psd=p)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name, fn in (('info', cmd_info), ('rate', cmd_rate),
                     ('stall', cmd_stall), ('noise', cmd_noise)):
        p = sub.add_parser(name)
        p.add_argument('port')
        p.add_argument('--baud', type=int, default=115200)
        p.add_argument('--seconds', type=float, default=30.0)
        p.set_defaults(fn=fn)
        if name == 'stall':
            p.add_argument('--stall', type=float, default=3.0)
            p.add_argument('--before', type=float, default=5.0)
            p.add_argument('--after', type=float, default=8.0)
        if name == 'noise':
            p.add_argument('--save', help='write the full PSD to this .npz')
    args = ap.parse_args()
    args.fn(args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
