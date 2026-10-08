"""
Test sequence runner — the step state machine, with no Qt and no timer.

The caller owns the clock and calls `tick(dt)`; this module owns which step is
current, how long it has run, and therefore what gain the test is demanding
right now. Outside a test the demand gain is 0 dB — but stopping the drive on
Stop / completion is the caller's job, so "0 dB" is never driven by accident.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from control import psd_grms

Steps = Sequence[tuple[float, float]]   # (duration_s, gain_db)


class SequenceRunner:
    IDLE     = 'idle'
    RUNNING  = 'running'
    PAUSED   = 'paused'
    COMPLETE = 'complete'

    def __init__(self, steps: Steps = ()) -> None:
        self._steps: list[tuple[float, float]] = [tuple(s) for s in steps]
        self._state     = self.IDLE
        self._step_idx  = 0
        self._elapsed_s = 0.0

    # ── State ─────────────────────────────────────────────────────────────────

    @property
    def state(self) -> str:
        return self._state

    @property
    def in_progress(self) -> bool:
        """A test is underway — running or paused."""
        return self._state in (self.RUNNING, self.PAUSED)

    @property
    def is_paused(self) -> bool:
        return self._state == self.PAUSED

    @property
    def steps(self) -> list[tuple[float, float]]:
        return list(self._steps)

    @property
    def step_index(self) -> int:
        return self._step_idx

    @property
    def step_elapsed_s(self) -> float:
        return self._elapsed_s

    @property
    def elapsed_s(self) -> float:
        """Elapsed across the whole sequence — step_elapsed_s alone is per-step."""
        done = sum(d for d, _ in self._steps[:self._step_idx])
        return float(done + self._elapsed_s)

    @property
    def total_duration_s(self) -> float:
        return float(sum(d for d, _ in self._steps))

    @property
    def gain_db(self) -> float:
        """Demand gain right now. 0 dB whenever no test is in progress."""
        if not self.in_progress or self._step_idx >= len(self._steps):
            return 0.0
        return self._steps[self._step_idx][1]

    # ── Transitions ───────────────────────────────────────────────────────────

    def start(self) -> bool:
        if not self._steps:
            return False
        self._state     = self.RUNNING
        self._step_idx  = 0
        self._elapsed_s = 0.0
        return True

    def pause(self) -> None:
        if self._state == self.RUNNING:
            self._state = self.PAUSED

    def resume(self) -> None:
        if self._state == self.PAUSED:
            self._state = self.RUNNING

    def stop(self) -> None:
        self._state     = self.IDLE
        self._step_idx  = 0
        self._elapsed_s = 0.0

    def tick(self, dt: float = 1.0) -> bool:
        """Advance the clock. True if the step changed or the sequence completed."""
        if self._state != self.RUNNING:
            return False
        if self._step_idx >= len(self._steps):
            self._state = self.COMPLETE
            return True
        self._elapsed_s += dt
        if self._elapsed_s < self._steps[self._step_idx][0]:
            return False
        self._step_idx += 1
        self._elapsed_s = 0.0
        if self._step_idx >= len(self._steps):
            self._state = self.COMPLETE
        return True

    def set_steps(self, steps: Steps) -> bool:
        """Replace the sequence, mid-test included.

        True if that ended a test in progress — the current step no longer
        exists, so the only safe reading is that the sequence is over.
        """
        self._steps = [tuple(s) for s in steps]
        if self.in_progress and self._step_idx >= len(self._steps):
            self._state = self.COMPLETE
            return True
        return False

    # ── Plan ──────────────────────────────────────────────────────────────────

    def plan(self, breakpoints) -> tuple[np.ndarray, np.ndarray]:
        """Target Grms staircase for the entire sequence, computed up front.

        Each step's Grms is the analytic integral of the profile at that step's
        gain, so the whole demand history is known before the test starts.
        """
        ts: list[float] = []
        gs: list[float] = []
        t = 0.0
        for dur, gain in self._steps:
            g = psd_grms(breakpoints, gain)
            ts.extend((t, t + dur))    # flat within the step, vertical at the edge
            gs.extend((g, g))
            t += dur
        return np.asarray(ts, dtype=np.float64), np.asarray(gs, dtype=np.float64)
