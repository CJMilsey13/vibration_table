# IN SPEC / ALARM / ABORT readout reads ABORT on a perfectly on-target signal

**Severity:** Medium (the readout cannot tell a good rig from a bad one)
**Location:** `imu_visualizer.py:1981-1985` (`_on_fft_done`), `1231-1238` (`update_spec_status`)
**Status:** Reproduced by running the code; confirmed by reading.

## Problem

The status compares raw per-bin Welch values from a 2 s window (1 s segments, 50 % overlap, about 3 averages) against ±3 dB and ±6 dB, with thresholds of 5 % and 2 % of bins. An estimate with that few averages has a per-bin scatter of about 2.9 dB by itself, so the thresholds are exceeded by estimator noise alone.

## Reproduction

Gaussian noise exactly on the 20-2000 Hz, 5e-4 g²/Hz profile, through the same filter and Welch settings:

| | result |
|---|---|
| bins outside ±3 dB | 26.1 % (ALARM above 5 %) |
| bins outside ±6 dB | 4.3 % (ABORT above 2 %) |
| trials reading ABORT | 100 % of 60 |
| "Error RMS" readout | 2.87 dB (green threshold is 3 dB) |

## Suggested fix

Judge tolerance on an estimate with enough degrees of freedom. Options:

- Use the EMA-smoothed display PSD for the status only (never for control).
- Average bins into fractional-octave bands before comparing.
- Widen the limits by the estimator's known confidence interval.

The "Error RMS" colour thresholds have the same problem: the noise floor of the readout sits at the green limit.

## Acceptance

An exactly on-target synthetic signal reads IN SPEC in at least 95 % of trials; a signal 6 dB off reads ABORT.

## Resolution (2026-10-07)

**Fixed.**

Verdict is taken on a running average of the last N control windows (`SpecAssessor`); N is set on the new Settings tab, default 8. No verdict is shown until N windows are in. "Error RMS" now reports the smoothed error the loop acts on (about 0.7 dB on a perfect signal).

Tests: `test_control.py::test_perfect_signal_reads_in_spec` (>= 95 % of 40 trials), `::test_real_excursions_are_still_caught`, `::test_gain_steps_do_not_invalidate_the_average`, `::test_error_readout_is_low_on_a_perfect_signal`; `test_gui.py::test_settings_tab_holds_the_in_spec_averaging`
