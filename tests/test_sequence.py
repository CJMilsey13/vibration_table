"""SequenceRunner: the step state machine behind tickets 02 and 12."""

import numpy as np
import pytest

from control import psd_grms
from sequence import SequenceRunner

STEPS   = [(3.0, -6.0), (2.0, 0.0)]
PROFILE = [(20.0, 0.0005), (2000.0, 0.0005)]


def test_idle_demands_zero_db_and_start_needs_steps():
    r = SequenceRunner([])
    assert r.state == r.IDLE and r.gain_db == 0.0
    assert not r.start() and not r.in_progress


def test_walks_the_steps_and_completes():
    r = SequenceRunner(STEPS)
    assert r.start() and r.gain_db == -6.0
    assert [r.tick(), r.tick()] == [False, False]
    assert (r.step_index, r.step_elapsed_s, r.elapsed_s) == (0, 2.0, 2.0)
    assert r.tick() is True                          # 3 s → step 2
    assert (r.step_index, r.gain_db, r.elapsed_s) == (1, 0.0, 3.0)
    assert r.tick() is False
    assert r.tick() is True
    assert r.state == r.COMPLETE and not r.in_progress
    assert r.gain_db == 0.0
    assert r.tick() is False                         # nothing more to do


def test_pause_freezes_the_clock_and_holds_the_gain():
    r = SequenceRunner(STEPS)
    r.start(); r.tick()
    r.pause()
    assert r.is_paused and r.in_progress and r.gain_db == -6.0
    assert r.tick() is False and r.step_elapsed_s == 1.0
    r.resume()
    r.tick()
    assert r.step_elapsed_s == 2.0


def test_stop_returns_to_idle():
    r = SequenceRunner(STEPS)
    r.start(); r.tick(); r.tick(); r.tick()
    r.stop()
    assert (r.state, r.step_index, r.elapsed_s, r.gain_db) == (r.IDLE, 0, 0.0, 0.0)


def test_editing_the_current_step_takes_effect_at_once():
    r = SequenceRunner(STEPS)
    r.start(); r.tick()
    assert r.set_steps([(3.0, -12.0), (2.0, 0.0)]) is False
    assert r.gain_db == -12.0 and r.in_progress


def test_deleting_the_step_being_run_ends_the_test():
    """Ticket 12: this used to leave the timer ticking forever."""
    r = SequenceRunner(STEPS)
    r.start()
    for _ in range(3):
        r.tick()
    assert r.step_index == 1
    assert r.set_steps(STEPS[:1]) is True
    assert r.state == r.COMPLETE and r.gain_db == 0.0


def test_shortening_the_current_step_advances_on_the_next_tick():
    r = SequenceRunner([(10.0, -6.0), (5.0, 0.0)])
    r.start(); r.tick(); r.tick(); r.tick()
    r.set_steps([(2.0, -6.0), (5.0, 0.0)])
    assert r.tick() is True and r.step_index == 1


def test_edits_while_idle_never_complete_anything():
    r = SequenceRunner(STEPS)
    assert r.set_steps([]) is False and r.state == r.IDLE


def test_plan_is_the_whole_staircase():
    t, g = SequenceRunner(STEPS).plan(PROFILE)
    assert list(t) == [0.0, 3.0, 3.0, 5.0]
    assert g == pytest.approx([psd_grms(PROFILE, -6.0)] * 2 + [psd_grms(PROFILE, 0.0)] * 2)
    assert SequenceRunner(STEPS).total_duration_s == 5.0
    t, g = SequenceRunner([]).plan(PROFILE)
    assert len(t) == 0 and isinstance(t, np.ndarray)
