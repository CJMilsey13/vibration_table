# Measured Grms sums control channels while the loop controls their mean

**Severity:** Medium (readout disagrees with demand on a correctly controlled test)
**Location:** `imu_visualizer.py:1933-1937` (Grms), `1955-1957` (mean PSD for control)
**Status:** Reproduced by running the code; confirmed by reading.

## Problem

The Grms readout sums power across every selected control channel. The loop controls the mean PSD of those channels. With two or more channels selected the two disagree by √N even when the rig is exactly on target.

The sum also covers the full 5-4000 Hz plot mask, while the demand Grms (`psd_grms`) integrates only the profile band.

## Reproduction

Accel X and Accel Z both exactly on the profile: "Meas: 1.399 g" against a demand of 0.995 g (√2, +3 dB). Three channels give √3. The Grms timeline then plots Measured at the edge of its ±3 dB band.

## Suggested fix

Compute Grms from the same `meas_psd` the loop uses (mean across channels), integrated over the profile band only.

## Acceptance

With any number of on-target control channels, measured Grms equals demand Grms within 2 %.

## Resolution (2026-10-07)

**Fixed.**

Grms readout, in-spec status and loop all read one mean PSD across the control channels; Grms is integrated over the profile band.

Tests: `test_gui.py::test_measured_grms_matches_demand_for_any_number_of_channels` (1, 2 and 3 channels), `test_control.py::test_band_grms_matches_demand_on_target`
