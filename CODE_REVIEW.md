# Code review — vibration table

**Date:** 2026-10-07
**Reviewed:** `imu_visualizer.py` (working tree, uncommitted changes included) and `firmware/main.c`, on top of commit `5b23d89`.

## Result

16 tickets: 3 high, 10 medium or low-medium, 2 that need checking on hardware or against the datasheet, 1 cleanup.

The three high-severity tickets share one effect: the shaker can be driven harder than the operator commanded.

1. With the loop enabled and the drive off, the level servo winds the output to full scale in about 8 s. The next Drive On applies it.
2. Pressing Stop leaves the drive running and raises the demand to the 0 dB profile.
3. The `speaker_response.json` on disk holds corrections up to +120 dB and is applied at startup with no clamp.

Fix these before the next run on the rig. Until then: keep the loop disabled while the drive is off, stop the drive by hand before pressing Stop, and delete `speaker_response.json`.

## Fix status (updated 2026-10-08)

All 16 tickets are fixed. One new ticket, 17, is open.
Each ticket file ends with a Resolution section naming its tests and measurements.

| # | Status |
|---|---|
| 01–07, 09–13, 16 | Fixed 2026-10-07. Covered by `python -m pytest tests` (123 tests, no hardware needed). |
| 08 | Fixed and measured on the rig 2026-10-08. A host stall is fully reported; the stale 512 ms at first connection is gone. |
| 14 | Fixed and measured on the rig 2026-10-08. The sensor runs at 8107.4 Hz, not 8000; the old firmware skipped about 107 samples a second. Now one read per sample, and the host resamples to exactly 8000 Hz. |
| 15 | Fixed as far as the sensor allows, measured on the rig 2026-10-08. Response at 1.4–2 kHz went from −9.2 dB to −1.9 dB. |
| 17 | **Open.** The remaining −1.9 dB at 1.4–2 kHz cannot be configured away at 8 kHz output rate. Options are in the ticket. |

Firmware `2026-10-08` is flashed on the development unit. The firmware it replaced is
`firmware/build/icm42688_streamer.uf2` (May 2026). The new firmware must be used with
the visualizer from the same commit: an older visualizer would show every frequency
1.33 % low.

The Python logic moved out of the widgets into three Qt-free modules: `control.py`,
`stream.py` and `sequence.py`. CLAUDE.md describes them and who owns which state.

### Found while fixing

- **A sequence step was applied twice in closed loop.** The step's gain reached the DAC
  correctly, but the next control update used a window measured mostly before the step,
  read it as a 5 dB shortfall and added it again: a +5 dB overshoot with clipping. The
  same happened at Drive On with the loop already enabled. Fixed: every drive change the
  loop did not make itself restarts the control window. This was caught by the
  end-to-end simulation, not by the review.
- **The documented board name does not exist.** `pimoroni_pico_plus2` fails to configure
  on SDK 2.2.0; the name is `pimoroni_pico_plus2_rp2350`. CLAUDE.md is corrected. The
  existing `firmware/build` directory is configured for `pico2`, a different board.
- **The bandpass filter starts as if every axis read 1 g.** Not changed. X and Y get a
  start-up transient near 5 Hz in the first window after connect. It is below the
  default 20 Hz profile edge, so it is noted here and not ticketed.

### How the fixes were verified

- Module tests run the controller against a simulated rig whose every measurement is
  real noise through the application's own Welch settings.
- `tests/test_closed_loop.py` runs the real window through a whole two-step sequence
  against a separate time-domain rig, using the real audio synthesis. It ends in spec at
  both steps, within 3 % of demand Grms, with no clipping.
- The extracted shape law was compared with the original inline code on identical
  inputs over 100 updates, across both rails and the unwind path: identical to the last
  bit.
- The suite passes on seven different noise seeds.
- The app was started in demo mode with its real threads. No audio was played and no
  rig was driven.

## How it was tested

- The Python findings were reproduced by running the real functions and the real `MainWindow` with synthetic data. No hardware was attached.
- Each finding was then checked a second time by reading the code at the cited lines. The numeric tests were re-run and gave the same figures.
- The saved response file was checked separately: min -22.3 dB, max +120.0 dB, mean 103.1 dB over 20-2000 Hz.
- The firmware was read, not built or flashed. Tickets 08, 14 and 15 have no hardware evidence.
- No repo code was changed. The test scripts are in the session scratchpad, not in the repo.

## Tickets

Each ticket is a file in `issues/` with location, reproduction, suggested fix and acceptance test.

### High — rig can be driven harder than commanded

| # | Ticket | Summary |
|---|---|---|
| 01 | [Level servo winds up with drive off](issues/01-level-servo-winds-up-with-drive-off.md) | Loop on, drive off: level goes -20 -> 0 dB in 4 updates. Next Drive On starts at full scale, 31.9 % of samples clipping. Reset does not clear it. |
| 02 | [Stop leaves drive running and raises level](issues/02-stop-leaves-drive-running-and-raises-level.md) | Stop and completion only stop the timer. Demand reverts to 0 dB, so the servo raises the drive: +6.8 dB after a -6 dB step, +12 dB after a -12 dB step. |
| 03 | [Saved speaker response loaded unclamped](issues/03-saved-speaker-response-loaded-unclamped.md) | `_load_response` applies no clamp. The file on disk spans -22 to +120 dB; with it, 94 % of drive power lands in 1-2 kHz and 20-1000 Hz gets 5.9 % instead of 49 %. |

### Medium — wrong behaviour or wrong readout

| # | Ticket | Summary |
|---|---|---|
| 04 | [Sequence gain steps have no effect open loop](issues/04-sequence-gain-steps-have-no-effect-open-loop.md) | Unit-RMS normalisation divides `gain_db` out. Block RMS is 1.000000 at -12, -6, 0 and +6 dB. The UI shows a step that does not happen. |
| 05 | [Reconnect freezes spectrum and control](issues/05-reconnect-freezes-spectrum-and-control.md) | Disconnect resets `_n_samples` but not `_last_welch_n` / `_last_ctrl_n`. After a 60 s session the first Welch fires at 60.03 s, not 2.0 s. |
| 06 | [In-spec status always reads ABORT](issues/06-in-spec-status-always-reads-abort.md) | Raw 3-average Welch bins scatter about 2.9 dB. A perfect signal has 26 % of bins outside ±3 dB and reads ABORT in 100 % of trials. |
| 07 | [Level servo bias runs rig hot](issues/07-level-servo-bias-runs-rig-hot.md) | Mean of dB errors is biased +0.78 dB on an on-target signal. The rig settles about +0.8 dB hot (+20 % power) with the error reading zero. |
| 08 | [Firmware overruns invisible to host](issues/08-firmware-overrun-drops-are-invisible-to-host.md) | `seq` is stamped after the ring pop, so samples dropped by `ring_push` leave no gap. Every connect delivers 512 ms of stale data, then an unreported gap. Read only. |
| 09 | [CLIP indicator clears during sustained clipping](issues/09-clip-indicator-clears-during-sustained-clipping.md) | Signal fires once on entry; label clears after 2 s. 200 clipping blocks emit one signal. |
| 10 | [Measured Grms sums control channels](issues/10-measured-grms-sums-control-channels.md) | Readout sums channels, loop controls their mean. Two on-target channels read 1.399 g against 0.995 g demand. |

### Low-medium

| # | Ticket | Summary |
|---|---|---|
| 11 | [Duplicate breakpoint frequency crashes profile update](issues/11-duplicate-breakpoint-frequency-crashes-profile-update.md) | `psd_grms` raises `ZeroDivisionError`; audio profile and target curve keep the old table while the loop uses the new one. |
| 12 | [Sequence edits not propagated](issues/12-sequence-edits-not-propagated.md) | Sequence edits do not emit `profile_changed`, so the Grms plan goes stale. Deleting a row mid-test leaves the test running forever. |
| 13 | [Audio and serial open failures swallowed](issues/13-audio-and-serial-open-failures-swallowed.md) | Open errors are only printed. The UI shows Drive On / connected with a dead worker. |

### Needs checking before work starts

| # | Ticket | Summary |
|---|---|---|
| 14 | [Firmware sampling not tied to DRDY](issues/14-firmware-sampling-not-tied-to-drdy.md) | Polling on the MCU timer is not locked to the sensor clock, so samples will repeat or skip. Code path confirmed; effect not measured. INT1 defines are unused. |
| 15 | [Sensor filters left at defaults](issues/15-firmware-sensor-filters-left-at-defaults.md) | Accel anti-alias and UI filters are never configured. If the defaults roll off below 2 kHz the loop over-drives the top of the band. Register defaults quoted from memory — check the datasheet. |

### Cleanup

| # | Ticket | Summary |
|---|---|---|
| 16 | [Dead code and stale tooltip](issues/16-dead-code-and-stale-text-cleanup.md) | `_update_fs`, `_fs_history`, `_fft_done` unused; per-frame `monotonic()` calls unused; tooltip still says the FFT uses the full ring buffer. |

## Documentation that no longer matches the code

- CLAUDE.md, failure-mode row "Audio level not changing between steps": the cause given (Drive On clicked after Start) is wrong. See ticket 04.
- CLAUDE.md, Grms timeline: "editing the table or sequence updates the plan immediately" is true for the table only. See ticket 12.
- CLAUDE.md, test sequence: "audio reverts to 0 dB shape" on stop is now a level increase under the level servo. See ticket 02.
- `firmware/main.c` header says sampling is interrupt-driven. It is polled. See ticket 14.

## Not done

- **GitHub issues:** filed on 2026-10-08 as
  [CJMilsey13/vibration_table #1–#16](https://github.com/CJMilsey13/vibration_table/issues).
  Issue number N is ticket N. The fixes are on branch `fix/code-review-tickets`.
  #1–#16 are closed with their resolution notes. #17 is open.
- The shaker was never driven. Every rig measurement was of the sensor at rest; the
  closed loop has only been run against simulated rigs.
- Alias rejection above 4 kHz with the wider sensor filter was not measured.
- `speaker_response.json` was left on disk. The app now refuses to load it.
- The branch is not merged into `main`.
