# Level servo is biased: it settles with the true level about 0.8 dB above demand

**Severity:** Medium (systematic over-test of about +20 % power)
**Location:** `imu_visualizer.py:2001` (`common = float(np.mean(err_db[band]))`)
**Status:** Reproduced by running the code; confirmed by reading.

## Problem

The servo nulls the mean of the per-bin dB errors. The mean of `10·log10` of a low-averaging PSD estimate is below the log of its mean, so an on-target signal produces a positive `common` every update. The servo has unity gain and no deadband, so it drives the level up until the biased mean reads zero.

## Reproduction

Exactly on-target signal: `common` = +0.784 dB (sd 0.072) on every update. The servo drifted -20 -> -17.57 dB over 3 updates on on-target input.

At equilibrium the rig runs about +0.8 dB hot (+20 % power, +9 % Grms) while the dB error reads zero.

## Suggested fix

Compute the common-mode term from linear in-band power:

```python
common = 10.0 * np.log10(np.sum(demand_psd[band]) / np.sum(meas_psd[band]))
```

This is unbiased and matches what the Grms readout reports. Keep `err_shape` zero-mean by subtracting the dB mean, not this value, or re-centre it explicitly.

## Acceptance

On an exactly on-target synthetic signal, `common` averages 0.0 dB within ±0.1 dB over 60 updates.

## Resolution (2026-10-07)

**Fixed.**

Level error is computed from linear in-band power.

Tests: `test_control.py::test_level_error_is_unbiased_on_target` (|mean| < 0.1 dB over 60 updates), `::test_converged_rig_runs_at_demand_not_above_it`; `test_closed_loop.py` (Grms within 3 % of demand)
