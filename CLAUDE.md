# Vibration Table — Development Instructions

## Process rules (mandatory)
- Always perform testing on new code that is added.
- Assume things are easily broken — verify answers.
- Treat this development as aerospace: verify everything you do. Ideally two independent checks agree with each other before marking something correct.
- Run `python -m pytest tests` before and after every change (needs `pytest`; ~15 s, no hardware, no sound device). `TEST_SEED=<n>` reruns the statistical tests on different noise.
- Firmware and sensor behaviour are checked on the real hardware with `python tools/hw_check.py {info,rate,stall,noise} <port>` (read-only — it never drives the shaker). Run them with the rig still and the drive off after any firmware change.
- The two independent checks already in the suite are the module tests against `tests/sim.py` (a PSD-domain plant) and `tests/test_closed_loop.py` (the real window, real synthesis, a time-domain rig). A control change is not verified until both agree.

---

## Project overview

A two-part system for measuring and driving a random-vibration shaker table:

1. **Firmware** (`firmware/main.c`) — C, Pico SDK 2.2.0, targeting Pimoroni Pico Plus 2 (RP2350)
2. **Visualizer** (`imu_visualizer.py`) — Python, PyQt5 + pyqtgraph, runs on the host PC.
   The logic it used to hold inline now lives in three Qt-free modules, each tested
   through its own interface:
   - `control.py` — `SpectralController` (level servo + shape loop + saved response),
     `SpecAssessor` (in-spec status), PSD profile helpers
   - `stream.py` — `MeasurementStream` (ring buffers, bandpass, the two Welch gates)
   - `sequence.py` — `SequenceRunner` (the test-step state machine)

---

## Hardware

| Part | Role |
|------|------|
| Pimoroni Pico Plus 2 (RP2350) | Microcontroller |
| ICM-42688-P | 6-axis IMU, SPI bus |
| Host PC audio output (3.5 mm) | Shaker drive signal |

### ICM-42688-P SPI wiring (Pico SPI0)
| IMU pin | Pico pin | GPIO |
|---------|----------|------|
| VCC | 3.3 V | — |
| GND | GND | — |
| MISO | GP16 | 16 |
| CS | GP17 | 17 |
| SCK | GP18 | 18 |
| MOSI | GP19 | 19 |

### Critical hardware quirk — MUST NOT be changed
The ICM-42688-P accel ADC requires gyro LN mode to be active in order for the shared PLL to start.
Accel-only power modes (0x0C, 0x08) leave the PLL off; ADC output stays permanently at 0x8000.

**Fix**: always write `PWR_MGMT0 = 0x0F` (accel LN + gyro LN) and wait 50 ms before reading data.

---

## Firmware (`firmware/main.c`)

### Configuration constants
| Symbol | Value | Meaning |
|--------|-------|---------|
| `ICM_PWR_ACCEL_LN_GYRO_LN` | `0x0F` | Both modes on — **do not change** |
| `ICM_ACCEL_ODR_8K` | `0x03` | 8 kHz output data rate |
| `ICM_ACCEL_FS_SEL_16G` | `0x00` | ±16 g FSR |
| `ICM_ACCEL_AAF_DELT` / `_DELTSQR` / `_BITSHIFT` | `63` / `3968` / `3` | Accel anti-alias filter at 3979 Hz, its widest — reset default is 24 / 576 / 6 ≈ 1.2 kHz |
| `ICM_GYRO_ACCEL_CONFIG0_VALUE` | `0x01` | Accel UI filter BW = ODR/2 (reset default `0x11` = ODR/4) |
| `ODR_PERIOD_US` / `DRDY_QUIET_US` | `125` / `90` | Nominal sample period; SPI is left idle this long after each sample |
| `BANNER_EVERY` | `16000` | Frames between status lines (2 s) |
| `WRITE_BATCH` | `64` | Frames per USB write (Core 1) |
| `RING_SIZE` | `4096` | Shared SPSC ring buffer size (8 bytes/entry, 32 kB, 512 ms) |
| `FRAME_BYTES` | `10` | Bytes per frame |

### Wire protocol (firmware → host, USB CDC)
```
[0xAA][0x55] | seq uint16 LE | ax int16 LE | ay int16 LE | az int16 LE
= 10 bytes per sample, at the SENSOR's output data rate
```

**The frame rate is not 8000 Hz.** It is the sensor's own ODR — 8 kHz nominal, set by
the sensor's internal oscillator, and **8107.4 Hz on the development unit** (+1.34 %).
The firmware measures it against the MCU crystal and reports it; the host resamples to
exactly 8000 Hz (see "Sensor sample rate" below).

Between frames, every 2 s (and once more 0.5 s after the first), the firmware sends one
ASCII status line:

```
# icm42688_streamer 2026-10-08 drdy-polled odr=8107.411 reset=11,0D,30,40,62 now=01,0D,7E,80,3F
```

- `odr=` sensor sample rate in Hz, measured over 32768 samples (≈1 ppm). `0.000` for
  the first second after boot.
- `reset=` / `now=` the sensor's `GYRO_ACCEL_CONFIG0`, `ACCEL_CONFIG1`,
  `ACCEL_CONFIG_STATIC2`, `STATIC3`, `STATIC4` as read after soft reset and as configured.
- It is plain ASCII, so it holds no `0xAA` and cannot be taken for a frame. Any parser
  that synchronises on `0xAA 0x55` skips it unchanged.
- It is repeated, not sent once at connect: opening a COM port on Windows purges the
  receive buffer just after raising DTR, which discards the first few ms of data.
- A stream with no status line is firmware older than 2026-10-08: MCU-timed at exactly
  8000 Hz, default sensor filters.

`seq` counts samples **read from the IMU**, not frames sent. It is stamped on Core 0
(`acq_seq`, in `ring_push`) and carried through the ring, so a sample dropped on
overrun leaves a gap and the host's `Drops:` counter reports it. Numbering frames at
transmit time on Core 1 — as the firmware used to — makes every overrun invisible.

### Init sequence (do not reorder)
1. Write `REG_BANK_SEL = 0`
2. Soft-reset `DEVICE_CONFIG = 0x01`, wait 10 ms
3. Check `WHO_AM_I == 0x47` → blink 2 if fail
4. Write `INTF_CONFIG1 = 0x91` (PLL clock), wait 1 ms
5. Write `ACCEL_CONFIG0 = 0x03` (±16 g, 8 kHz) **before** enabling
6. Write `GYRO_CONFIG0 = 0x03` (±2000 dps, 8 kHz) **before** enabling
   - 6a. Read `GYRO_ACCEL_CONFIG0`, `ACCEL_CONFIG1` and (bank 2) `ACCEL_CONFIG_STATIC2/3/4`
     — the reset values, kept for the status line
   - 6b. Bank 2: write `ACCEL_CONFIG_STATIC2/3/4 = 0x7E / 0x80 / 0x3F` (AAF 3979 Hz),
     read back, **return to bank 0**
   - 6c. Write `GYRO_ACCEL_CONFIG0 = 0x01`, `ACCEL_CONFIG1 = 0x0D`, read back →
     blink 4 if any of 6b/6c did not stick
   - These were inserted here, not appended: bank-2 (static) registers may only be
     written while accel and gyro are **off**, i.e. before step 7.
7. Write `PWR_MGMT0 = 0x0F` (accel LN + gyro LN), wait **50 ms**
8. Readback `PWR_MGMT0` → blink 3 if fail
9. Poll `INT_STATUS` bit 3 (DRDY) up to 500 ms → blink 5 if timeout
10. Verify 20 DRDY cycles produce non-0x8000 accel data → blink 6/7 if fail

### Blink error codes
| Flashes | Meaning |
|---------|---------|
| 2 | WHO_AM_I mismatch |
| 3 | PWR_MGMT0 readback fail |
| 4 | Accel filter config readback fail (AAF / UI filter) |
| 5 | DRDY timeout |
| 6 | All axes stuck at 0x8000 |
| 7 | Temp valid but accel stuck at 0x8000 (PLL not running) |

### Dual-core architecture
- **Core 0**: reads **exactly one sample per sensor data-ready**. It polls `UI_DRDY`
  in `INT_STATUS` over SPI (reading it clears the flag), then burst-reads the accel
  registers and pushes to the ring. The bus is left idle for `DRDY_QUIET_US` after each
  sample, since the next DRDY cannot come sooner. No INT1 wire is needed or used.
- Core 0 also measures the sensor's rate (samples ÷ MCU time) for the status line, and
  if it is ever held up for two periods or more it advances `acq_seq` by the samples it
  missed, so they show as drops. Measured: 0 drops in 1.46 M frames.
- **Core 1**: reads ring buffer, packs 10-byte frames, batches 64 frames per `fwrite` to USB CDC (125 writes/s)
- Core 1 **discards the ring when the host first connects** (`ring_rd = ring_wr`). Core 0
  samples from boot, so without this every connection began with 512 ms of stale data.
- **Never go back to reading on an MCU timer** (`next += 125 µs`). The sensor's clock is
  not locked to the MCU's and runs 1.34 % fast: read at 8000 Hz, 107 of its 8107 samples
  a second were never read, and about 5 a second were read twice. Each is a one-sample
  jump in time. On the rig it put sidebands ±107 Hz either side of an ambient line at
  1737 Hz (2–3 dB over the floor beside a 13 dB line); with one read per DRDY they are
  gone (0.1–0.3 dB).
- Ring buffer is lock-free SPSC with `__dmb()` barriers

### Build
```
cmake -G Ninja -DPICO_BOARD=pimoroni_pico_plus2_rp2350 -DPICO_SDK_PATH=<sdk> ..
cmake --build .
```
The board is `pimoroni_pico_plus2_rp2350` in SDK 2.2.0 — plain `pimoroni_pico_plus2`
does not exist there and configure fails. The VS Code Pico extension's toolchain lives
under `~/.pico-sdk` (cmake, ninja, arm-none-eabi-gcc, picotool).

**What is actually flashed is a `-DPICO_BOARD=pico2` build**, and always has been
(`picotool info` on the May 2026 firmware says `pico_board: pico2`; the chip is an
RP2350B in QFN-80 with 16 MB flash, i.e. a Pico Plus 2). It works. The
`pimoroni_pico_plus2_rp2350` build compiles but has **never been run on the hardware** —
try it only with someone at the bench to press BOOTSEL if USB does not come up.

Flashing needs no button: opening the CDC port at 1200 baud reboots into BOOTSEL, then
`picotool load -v -x icm42688_streamer.uf2`. USB is brought up before the IMU is
initialised, so this should still work while the firmware is halted on a blink code
(not tested — no init fault has occurred). Keep that order: it is the only way back in
without pressing the button.
Outputs `icm42688_streamer.uf2` — drag-and-drop to flash.

---

## Python visualizer (`imu_visualizer.py`)

### Key constants
| Constant | Value | Notes |
|----------|-------|-------|
| `SAMPLE_RATE` | 8000.0 Hz | Rate of the stream **after resampling** — exact. The sensor's own rate is reported by the firmware and resampled away in `MeasurementStream` |
| `WINDOW_TIME` | 10.0 s | Max ring-buffer / display length |
| `HISTORY` | 80 000 samples | `SAMPLE_RATE × WINDOW_TIME` |
| `SPEC_N` | 8000 | Welch segment length = 1 s → 1 Hz resolution |
| `CTRL_WINDOW_S` | 2.0 s | Control measurement window — Welch covers only this, not `HISTORY` |
| `CTRL_SAMPLES` | 16 000 | `SAMPLE_RATE × CTRL_WINDOW_S` → 3 Welch averages per update |
| `WELCH_MIN_SAMPLES` | 16 000 | First PSD fires after 2 s, not 10 s |
| `DISPLAY_WELCH_HZ` | 15 Hz | Welch rate for the spectrum display (batch-quantised to ~14.4 Hz) |
| `PSD_DISPLAY_TAU_S` | 0.35 s | EMA time constant on the **displayed** PSD — never on control |
| `CTRL_SETTLE_S` | 0.25 s | Guard after an external drive change before the next control window starts |
| `TOL_ALARM_DB` / `TOL_ABORT_DB` | 3 / 6 dB | Tolerance bands drawn around the demand profile |
| `SPEC_AVG_DEFAULT` | 8 windows | Control windows averaged for the in-spec verdict (Settings tab, 1–32) |
| `RESPONSE_MAX_CLAMPED_FRAC` | 5 % | A saved response with more of its bins outside the clamp is refused |
| `LEVEL_MAX_DBFS_DEFAULT` | −12 dBFS | Highest RMS the level servo will send to the DAC (Settings tab, −20…0) |
| `CLIP_FRACTION` | 0.5 % | Share of a block's samples on the rail before `CLIP` is shown |
| `CTRL_DEADBAND_DB` | 0.5 dB | Soft deadband ≈1σ of the smoothed estimate |
| `CTRL_MAX_STEP_DB` | 6.0 dB | Per-update slew limit on the correction |
| `BATCH` | 80 | Samples per Qt signal emission |
| `PSD_FMIN` | 5.0 Hz | Bandpass + PSD lower bound |
| `PSD_FMAX` | 4000.0 Hz | Nyquist at 8 kHz |

Timing constants are defined in `stream.py`, control-law tuning in `control.py`;
`imu_visualizer.py` imports what it displays.

### Architecture
- `SerialWorker` (QThread) — reads 4096-byte chunks from USB CDC, parses 10-byte frames, emits `batch_ready(samples, drops)`; emits `info_ready(str)` for each firmware status line; emits `failed(str)` if the port cannot be opened or is lost
- `DemoWorker` (QThread) — synthetic 50 Hz + 120 Hz + 1 kHz signals for offline testing
- `WelchRunnable` (QRunnable) — Welch PSD on thread pool (50% overlap, Hann window, `scaling='density'`)
- `AudioOutputWorker` (QThread) — IFFT-shaped Gaussian noise → sounddevice OutputStream; `render()` makes one block and is what the tests call; emits `failed(str)` if the stream cannot be opened
- `MainWindow` — 30 Hz display timer, plots, and the glue: stream → Welch → controller → audio worker. Central widget is a tab pair, **Live** (plots) and **Settings**
- `TestProfileDock` (QDockWidget) — PSD breakpoint table, test sequence, the 1 Hz timer that ticks the `SequenceRunner`, the level slider, the audio worker
- `MeasurementStream` (`stream.py`) — `push(batch)`, `take_window()`, `restart_control_window()`, `reset()`, `set_input_rate(hz)`; holds a `Resampler` when the input is not at exactly 8000 Hz
- `SpectralController` (`control.py`) — `update(meas_psd, breakpoints, gain_db, drive_active)`, `reseed_level()`, `output_dbfs()`, `reset()`, `load_response()` / `save_response()`
- `SpecAssessor` (`control.py`) — `add(meas_psd, …)`, `status()` → `SpecStatus`
- `SequenceRunner` (`sequence.py`) — `start/pause/resume/stop/tick`, `gain_db`, `set_steps`, `plan`

**Who owns what** — each of these had two owners before, and each split produced a bug:

| State | Owner | Everyone else |
|---|---|---|
| Drive level | `SpectralController.level_db` | the slider only *re-seeds* it; the dock relays `output_dbfs()` to the worker |
| Correction + saved response | `SpectralController` | `MainWindow` pushes `corr_db` to the worker |
| Current step / demand gain | `SequenceRunner.gain_db` | 0 dB whenever no test is in progress |
| Sample counters, both Welch gates | `MeasurementStream` | reset together, on connect |

**One measured PSD.** `_on_fft_done` forms the mean PSD across the selected control
channels once, and the Grms readout, the in-spec status and the loop all read that same
array. They cannot disagree. (The readout used to *sum* channels while the loop used the
mean — √N apart with N channels.)

### PSD computation
- Welch uses `fs = SAMPLE_RATE = 8000.0` (constant). That is exact because the stream is resampled to it; the host never estimates the rate from serial timing itself — chunked reads make that unreliable over short spans (parsing time ≈ µs, not the 125 µs sample period)
- x-axis: pre-log10 frequencies passed to `LogHzAxis` — `setLogMode` must NOT be used (causes double log10)
- y-axis: **PSD is displayed in g²/Hz, not dB.** Curves are fed `np.log10(psd)` and
  `LogPSDAxis` formats the ticks back to plain g²/Hz (`1e-4`, `1e-3`, …). Same
  pre-log10 convention as the x-axis, so `setLogMode` must not be used here either.
- The control loop still works internally in **dB** — `err_db`, `H_corr_db` and the
  Max-corr clamp are all dB power. Only the display is g²/Hz. When plotting the drive
  curve the conversion is `log10(demand) + H_corr_db / 10`; the `/10` is not optional.
- `_plot_freqs` and `_plot_mask` use `np.fft.rfftfreq(SPEC_N, 1/fs)`, same bins as scipy Welch output

### Sensor sample rate — measured, reported, resampled

The ICM-42688-P's 8 kHz ODR comes from its internal oscillator. On the development
unit it is **8107.41 Hz** (firmware, against the MCU crystal) = 8107.8 Hz (PC clock) —
two clocks, agreeing to 50 ppm. It holds to ±0.005 Hz over minutes.

The firmware delivers every sensor sample once, so the stream arrives at that rate, and
`MeasurementStream` resamples it to exactly 8000 Hz before anything else sees it:

- `SerialWorker.info_ready` → `MainWindow._on_serial_info` → `banner_rate()` →
  `MeasurementStream.set_input_rate()`.
- The **first** report of a session changes the rate by more than
  `INPUT_RATE_RESET_FRAC` (0.1 %), so the stream is reset (the ≤0.5 s gathered so far was
  on the wrong time base) and `_psd` / the in-spec average are dropped. Later reports
  differ by a few ppm and are just followed.
- A new connection goes back to the nominal rate until that device reports its own, so
  older firmware (no status line, MCU-timed at 8000 Hz) still works unchanged.
- `Resampler` is windowed-sinc interpolation, 64 taps, Kaiser β = 9: tones keep their
  frequency to 0.02 Hz and their level to 0.01 dB up to 3 kHz, the passband is flat to
  3.3 kHz, and nothing above 4 kHz folds back (−89 dB). Above 3.3 kHz it rolls off —
  the display is not meaningful there. Cost ≈ 1 % of a core.
- The toolbar reads `FS: 8000 Hz (sensor 8107.4 Hz, resampled)`; the status line is its
  tooltip.

**Do not "simplify" this by relabelling the frequency axis or dropping the resampler.**
Unresampled, every frequency reads 1.33 % low (2000 Hz shows at 1973 Hz), so the top
27 Hz of a 20–2000 Hz profile never sees any drive and the loop pins those bins on the
boost rail. Verified on the rig with ambient lines: 1737 / 2606 Hz under the old
firmware, 1714 / 2571 Hz unresampled, 1737 / 2606 Hz again through the resampler.

### Sensor frequency response — measured

Measured from the at-rest noise floor (the sensor's own noise is white, so its shape is
the sensor's filtering; `python tools/hw_check.py noise <port>`). dB relative to
100–300 Hz, mean of the three axes:

| Band (Hz) | Reset-default filters | As configured |
|---|---|---|
| 300–600 | −0.4 | −0.1 |
| 600–1000 | −1.7 | −0.3 |
| 1000–1400 | −4.6 | −0.9 |
| 1400–2000 | **−9.2** | **−1.9** |
| 2000–3000 | −16.2 | −3.8 |
| 3000–3900 | −22.4 | −7.8 |

- The reset defaults (AAF ≈ 1.2 kHz, UI filter ODR/4) roll off well inside a 20–2000 Hz
  profile. The loop reads that as the rig's response and over-drives the top of the
  band by the same amount. They are therefore opened to their widest at init.
- **The remaining −1.9 dB at 1.4–2 kHz cannot be configured away at 8 kHz ODR.** UI
  filter order (1st/2nd/3rd), UI bandwidth code (0, 14, 15) and AAF on/off were all
  tried on the hardware: identical below 2 kHz. It belongs to the sensor's fixed
  decimation chain at this ODR. A flatter response needs the sensor run at 16 or 32 kHz
  and decimated in the MCU, or a correction on the host (issue #17).
- So the top third-octave of a 2 kHz profile is still driven ≈ 2 dB harder than
  indicated. Until #17 is done, treat readings above 1 kHz as that much low.
- The price of the wider AAF is alias rejection. Not measured — it needs a source above
  4 kHz — but from the in-band slopes, content at 6–8 kHz, which folds back into
  0–2 kHz, is probably attenuated by something like 15–20 dB now against 30 dB or more
  with the default filter. The drive is band-limited to the profile, so only harmonics
  and rattles live up there.
- DC gain is unaffected: |g| at rest reads 0.9997–1.0002 g with every setting tried.

### Spectrum display

Welch runs at `DISPLAY_WELCH_HZ` so the spectrum feels live; those windows overlap
heavily, which is fine for display. The **control update is gated separately** on
`CTRL_SAMPLES` of new data, so it still only ever sees non-overlapping windows.
Decoupling these two rates is what allows a fast display without reintroducing the
limit cycle — they must never be re-merged.

- **Tolerance bands** at ±`TOL_ALARM_DB` (orange dotted) and ±`TOL_ABORT_DB` (red
  dashed) are drawn across the profile band only, with an `IN SPEC` / `ALARM` /
  `ABORT` readout giving the share of in-band bins inside each. It is computed even
  when the loop is **off** — it reports what the rig is doing, not what the controller
  is doing.
- **The verdict is taken on a running average of the last N control windows**
  (`SpecAssessor`, N = `SPEC_AVG_DEFAULT`, set on the **Settings** tab). One 2 s window
  has ~3 Welch averages, so each bin scatters ~2.9 dB on its own: a *perfect* signal
  put 26 % of bins outside ±3 dB and read ABORT in 100 % of trials. Until N windows
  are in, the readout says `AVERAGING k/N` and gives no verdict. Per-bin scatter after
  N windows ≈ 2.9/√N dB (1.0 dB at N = 8).
- Each window is stored as measured ÷ demand, so a sequence gain step does not
  invalidate the average. It is reset on a profile edit, Drive On/Off, Reset, test
  start and reconnect. It is fed **control windows only** and is display-only — never
  feed it back into the loop.
- **Y-range is anchored to the demand profile**, not autoscaled. A bin on the
  correction clamp would otherwise drag the scale to 1e-15 and squash every real
  trace into the top of the pane. The floor drops adaptively (hysteresis 0.25
  decade, capped at `PSD_VIEW_MAX_DECADES`) when the **in-band** measurement runs
  far below target, so an open-loop run is still fully visible. Out-of-band roll-off
  is deliberately excluded from that calculation — it is not what the test is judged
  on, and letting it set the scale compresses the tolerance band for no reason.
- **Correction H(f) has its own right-hand dB axis**, not the PSD axis. It is not a
  PSD — the drive's absolute level is set by the Level slider after unit-RMS
  normalisation — and forcing it onto the left axis would wreck the tolerance
  scaling. Once converged this trace is the inverse of the rig's transfer function.

### Grms timeline

Third plot pane, below the spectrum: demand vs. achieved overall level over the test.

- **Demand is precomputed for the entire sequence before the test starts.** Each
  step's Grms is the analytic integral (`psd_grms`) of the profile at that step's
  gain, so the whole staircase is known up front — drawn as flat plateaus with
  vertical transitions at step boundaries. It is rebuilt on `profile_changed`, so
  editing the table or sequence updates the plan immediately.
- **Measured** is appended once per control window while the test runs. It is
  `band_grms` of the mean control-channel PSD over the **profile band** — the same span
  `psd_grms` integrates for demand — so an on-target rig reads on the demand line with
  any number of control channels.
- A **sequence** edit emits `sequence_changed`, which rebuilds the plan but does not
  reset the correction (the rig has not changed). A **breakpoint** edit emits
  `profile_changed`, which does both.
- x-axis is **cumulative** test time via `TestProfileDock.test_elapsed_s()`.
  `SequenceRunner.step_elapsed_s` alone is per-step and resets at every boundary — do
  not plot against it; `elapsed_s` is the cumulative one.

### Audio drive output
- `psd_interp_loglog`: log-log interpolation of breakpoints onto frequency bins.
  **It clamps frequency into the breakpoint range**, so it returns the *edge* level
  outside the profile — never zero. Callers must band-limit themselves.
- `_generate_shaped_block`: IFFT method — amplitude per bin = `sqrt(PSD(f) * df)`, random phase
- **MUST synthesise only inside `[max(PSD_FMIN, bp[0]), min(PSD_FMAX, bp[-1], fs/2)]`,
  plus half a drive bin (`fs/n/2` ≈ 5 Hz) of slack at each end.** The slack is there so
  the band is driven right to its edges: a drive bin is ~11 Hz wide, and stopping at the
  last bin *centre* inside the band left the top few Hz with no drive (−4.2 dB at
  1996–2000 Hz). The loop then wound those bins toward the boost rail — the spike at the
  right-hand end of the correction curve. A slack bin takes the band-edge correction
  (the lookup frequency is clamped into the band), never the 0 dB taper.
  Because the block is normalised to unit RMS, any out-of-band energy directly steals
  level from the in-band signal. Emitting to Nyquist on a flat 20–2000 Hz profile puts
  only 8.9 % of the power in-band (−10.5 dB).
- **Correction must taper to 0 dB outside the control band** (`left=0.0, right=0.0`).
  Holding the saturated edge value multiplies out-of-band power by `10^(max_corr/10)`.
- Signal normalised to unit RMS before `output_gain` scaling, so `output_gain` is the only amplitude control
- **A sequence step's gain is applied as output level, not in the block.** `gain_db`
  scales every bin alike and the normalisation divides it straight back out — block RMS
  is 1.000000 at −12, −6, 0 and +6 dB. The DAC level is
  `SpectralController.output_dbfs(gain_db)` = level + step gain, capped at 0 dBFS, set by
  `TestProfileDock._apply_level()` on every step. Open loop, a −6 → 0 dB step is exactly
  +6.00 dB at the DAC.
- The label beside the Level slider shows the level **going to the DAC** (slider/servo
  level + step gain), not the slider position.
- Clip detection: always hard-clips at ±1.0, and fires `clip_detected` for **every**
  block with more than `CLIP_FRACTION` (0.5 %) of its samples on the rail; each one
  restarts the 2 s clear timer, so the red "CLIP" stays lit for as long as clipping
  continues. One flattened 4σ peak in a block is not reported — that is what −12 dBFS
  looks like and it is 50 dB down. In practice `CLIP` means the drive is above about
  −10 dBFS RMS.

### Test sequence behaviour
- Clicking **▶ Start** in Test Control:
  1. Starts the 1 Hz step timer
  2. Auto-starts the audio drive (Drive On) if not already running
  3. Sets the drive gain to step 0's gain_db immediately
- Each step advance calls `_enter_step`, which applies the step's gain at the DAC
  (`_apply_level`) and emits `drive_changed`
- **Stop and completion both stop the drive.** Outside a test the demand is the 0 dB
  profile; leaving drive and loop running raised the shaker +6.8 dB after Stop on a
  −6 dB step. Do not make the drive outlive the test.
- **Pause is a hold**: the drive keeps its level and shape, the step clock stops, and
  the loop does not integrate (`drive_active=False`).
- **Drive On** can also be used independently without running a test sequence. It
  always re-seeds the level from the slider — never from where the servo was left.
- If the audio stream fails, the drive is marked off, a running test is stopped, and
  the status bar says why.
- Deleting the step being run ends the test; editing its gain moves the drive at once.

### Closed-loop spectral control

The controller runs a per-frequency-bin integral feedback loop after every Welch result (~2 s cycle):

```
err_dB[f]     = 10·log10(demand[f]) − 10·log10(measured[f])   # signed power error per bin
H_corr_dB[f] += loop_gain × err_dB[f]                          # integral accumulation
H_corr_dB[f]  = clip(H_corr_dB, −max_corr, +max_corr)          # stability clamp
```

The correction `H_corr_dB` is applied inside `_generate_shaped_block` by interpolating it onto the audio FFT grid (log-spaced) and scaling `psd_vals` by `10^(H_corr/10)` before computing amplitudes. This multiplies the drive PSD exactly where the measured PSD is below demand and reduces it where it overshoots.

**Key invariants:**
- `H_corr_db` is stored in Welch frequency space (1 Hz resolution, 5–4000 Hz)
- Correction is interpolated to the audio block grid (~10.8 Hz at 44100/4096) via log-linear `np.interp`
**The loop must update exactly once per fresh, non-overlapping measurement.**
This is the single most important invariant in the controller. Welch runs over the
most recent `CTRL_SAMPLES` (2 s) only, and `_on_batch` launches it only once that
many *new* samples have arrived, so consecutive updates share no data.

Previously Welch covered the whole 10 s ring and was relaunched on every batch, so
the loop ran at ~100 Hz against a 10 s moving average — ~1000 updates before the
measurement reflected even one of them. Every gain setting saturated to an effective
gain of 1.0 per measurement refresh, winding the correction to the clamp and back:
a sustained limit cycle at ~2× the window, rail to rail. Lowering `loop_gain` could
not fix it because the gain was not the variable that mattered.

- The plant is **memoryless in dB** (`measured_dB = drive_dB + plant_dB`), so a plain
  integrator is the correct structure — error decays as `(1 − gain)^n`, deadbeat at
  gain 1, stable below 2. No PID needed; derivative would only amplify noise.
- `loop_gain = 0.5` default, range 0.05–1.0. Simulated: 0.3 → settles 14 s / 0.27 dB
  ripple; 0.5 → 8 s / 0.65 dB; 0.7 → 4 s / 0.85 dB; 1.0 → 4 s / 1.41 dB.
- Error is smoothed across frequency (`gaussian_filter1d`, sigma = 5 bins) so one
  noisy bin cannot swing its own correction independently of its neighbours.
- `CTRL_DEADBAND_DB = 0.5` — **soft**-thresholded (`sign(e)·max(|e|−d, 0)`), not
  hard-gated, so the correction stops random-walking on measurement noise once
  converged without chattering at the threshold.
- `CTRL_MAX_STEP_DB = 6.0` — per-update slew limit, so one bad window cannot slam a
  bin to the rail.
- **The clamp is asymmetric.** `max_correction_db` sets the *boost* limit only;
  cutting is capped separately at `CTRL_MAX_CUT_DB` (40 dB). See below.
- **Anti-windup by conditional integration**: a step is zeroed if the bin is already
  on the rail it would push further into. Without this a bin the rig cannot reach
  keeps integrating, and has to unwind through tens of dB before the drive responds
  when conditions change.

### Why the clamp is asymmetric — the diagonal-plant assumption

The controller assumes `measured_dB[f] = drive_dB[f] + plant_dB[f]`: each bin
responds only to its own drive. A resonant structure violates this. A lightly-damped
mode is excited by energy *anywhere* in the band, so the measured level at the
resonance does not fall when you cut the drive there.

When that happens the loop cannot null the error by cutting, so a symmetric clamp
lets the integrator run to the negative rail chasing a term it can never reach —
observed in the field as the correction pinned at −120 dB across a resonant region
while the measured PSD stayed *above* target.

Simulated against a non-diagonal plant (resonance fed by the rest of the band):

| | correction spread | unwind debt | recovery |
|---|---|---|---|
| symmetric, no anti-windup, 120 dB | 108.7 dB | 94.5 dB | 34 s |
| symmetric, no anti-windup, 200 dB | 109.6 dB | 95.5 dB | 34 s |
| **asymmetric 40 dB + anti-windup** | **54.2 dB** | **40.0 dB** | **16 s** |

Two things to note. Raising the ceiling makes the *old* behaviour slightly worse and
never better — the runaway is at the negative rail, which the ceiling does not
govern. And a 109 dB spread puts the working bins past 16-bit DAC resolution, so
they degrade to dither while the loop chases an uncontrollable bin.

Cutting past ~40 dB is never useful: the bin is already switched off. If it is still
too loud the energy is cross-coupled, and `sat %` will say so — that is the signal to
damp the fixture or narrow the profile, not to raise the clamp.

### Why 40 dB — the measurement floor, not the DAC

The binding constraint is the **accelerometer**, not the converter:

| | g²/Hz |
|---|---|
| ICM-42688-P noise density, 65 µg/√Hz | 4.23e-9 |
| ADC quantisation at ±16 g | 4.97e-12 (29 dB lower — not the limit) |
| demand at 0 dB / −6 dB / −12 dB | 5.0e-4 / 1.26e-4 / 3.16e-5 |

That gives **50.7 / 44.7 / 38.7 dB** of headroom above the sensor's own noise. A bin
cut 40 dB at the −6 dB step sits ~4.7 dB above the noise floor; cut it further and
the loop is measuring the sensor's noise and correcting against it. This — not DAC
bit depth — is what sets `CTRL_MAX_CUT_DB`. Raising it requires a quieter
accelerometer, not a better sound card.

The output is already `float32` (24-bit mantissa) into the audio API, which is the
most any host accepts; synthesis is float64 internally. Converter hardware tops out
near 24 effective bits (~120 dB) because Johnson noise in the analog stage dominates
— 32-bit interfaces do not deliver their theoretical 194 dB, and 64-bit converters do
not exist.

### Two loops: level and shape

**The per-bin correction cannot change overall level.** `_generate_shaped_block`
normalises every block to unit RMS, so a common-mode (uniform) correction is divided
straight back out — measured at 0.00 dB of output change for uniform corrections of
20, 60, 120 and 200 dB alike. Only the *deviation from the mean* survives.

Control is therefore split on every update:

```
common    = 10·log10(Σ demand[band] / Σ measured[band])   -> level servo
err_shape = err_dB − mean(err_dB[band])                    -> per-bin integral loop
```

**The level error is taken from linear in-band power, not from the mean of the per-bin
dB errors.** The mean of `10·log10` of a ~5-DOF PSD estimate sits below the log of its
mean: on a perfectly on-target signal the dB mean reads +0.78 dB every update, and a
servo nulling it settles with the rig +0.8 dB hot (+20 % power). The linear ratio is
unbiased (|mean| < 0.1 dB over 60 updates) and is what the Grms readout shows. The shape
error is still made zero-mean in dB — that is what keeps a level error out of the shape
loop.

- **Level servo** trims `SpectralController.level_db`, slew-limited to
  `CTRL_LEVEL_MAX_STEP_DB` (6 dB) because it is a real physical level change. The Level
  slider is the *starting point*, not a fixed setting; moving it, Drive On and Reset all
  re-seed the servo. `level_db` is referred to the 0 dB profile; the DAC gets
  level + step gain, and the servo stops at the rail that keeps that ≤ 0 dBFS.
- **Shape loop** now sees a zero-mean error, so it can only ever do shape work.

Before the split, a pure level deficit was fed to the per-bin loop, which is blind
to it — so every bin integrated toward the clamp chasing an error it structurally
could not fix. Observed in the field as the correction pegged at +200 dB across the
band with the measured Grms flat for ~22 s. Simulated, rig starting 9 dB low:

| | cold start | +6 dB sequence step | peak correction |
|---|---|---|---|
| single per-bin loop | **never** (−15.7 dB residual) | **never** | 200 dB, pinned |
| **split level + shape** | **6 s** (0.05 dB) | **2 s** | **3.5 dB** |

If the loop ever appears to "do nothing for tens of seconds", check whether the
correction is winding uniformly — that is the signature of a level error reaching
the shape loop, and it means the split has been broken.

### The drive ceiling — the level servo must not drive into clipping

The drive is Gaussian noise, so its peaks run far above its RMS. At an RMS of L dBFS
everything beyond `10^(−L/20)` σ is flattened by the DAC:

| RMS level | clips at | samples clipped | distortion below the drive |
|---|---|---|---|
| 0 dBFS | 1.0 σ | 31.7 % | 10 dB |
| −6 dBFS | 2.0 σ | 4.6 % | 20 dB |
| −9.5 dBFS | 3.0 σ | 0.28 % | 34 dB |
| **−12 dBFS** | 4.0 σ | 0.007 % | 52 dB |

(`control.gaussian_clip`, checked against a brute-force clip of 2 M samples.)

**Clipping distortion is broadband — it lands in every bin whatever that bin was asked
to carry.** This rig's response spans about 35 dB (a resonance near 80 Hz against a weak
top end), so flattening it means cutting the drive ~30 dB at the resonance. Distortion
only 10–20 dB below the drive then sets the level at the resonance, not the loop. That
is the "energy the loop cannot cut" described under the asymmetric clamp — made by the
drive itself.

So the level servo stops at `SpectralController.max_output_dbfs` (default −12 dBFS,
**Settings → Max drive level**), and `output_dbfs()` never exceeds it whatever the
slider or a sequence step asks for. When the servo is on that ceiling and the rig is
still below demand, `ControlUpdate.at_limit` is set and the dock shows, in red:

```
DRIVE AT LIMIT (−12 dBFS) — rig is 14.1 dB short. Turn the amplifier up.
```

The figure is the level error at the ceiling: exactly how much more gain the amplifier
has to supply. The Level label turns orange whenever the ceiling is holding the output
below what the slider + step gain asked for.

Seen in the field (`1008 debug 2.png`): Level `+0.0 dB`, `CLIP`, measured Grms flat at
0.28 g through −12 / −8 / −6 dB steps demanding 0.35 / 0.56 / 0.71 g, resonance band the
hottest part of the spectrum although its correction was the deepest cut. Reproduced in
`tests/test_closed_loop.py` with the real synthesis and real clipping against a rig with
a 35 dB resonance and too little gain:

| | ceiling 0 dBFS (old) | ceiling −12 dBFS | −12 dBFS, amplifier +15 dB |
|---|---|---|---|
| Measured at the −12 / −8 / −6 dB steps (demand 0.353 / 0.560 / 0.705 g) | 0.35 / 0.43 / 0.42 | 0.14 / 0.14 / 0.14 | 0.352 / 0.561 / 0.702 |
| Samples clipped | 23 % | 0.005 % | 0.0001 % |
| Resonance vs. rest of band | **+8 dB**, cut on the −40 dB rail | +0.3…+1.2 dB | +0.3…+1.2 dB |
| Readout | CLIP, `sat` | `DRIVE AT LIMIT … 14 dB short` | in spec |

Two things to read from that table. Driving into clipping does buy level (0.42 g against
0.14 g) — but not the level asked for, and it loses the shape. And the steps add nothing
in either limited case: once the output is on a ceiling, a step's feedforward is clamped.
**A flat Grms across sequence steps means the drive is at a limit, not that the step
logic is broken.**

- Do not raise the default ceiling to "get more power". The fix for too little level is
  amplifier gain, or a lower / narrower profile.
- `max_boost_db` (Max corr) limits **boost only**. The cut rail is always
  `CTRL_MAX_CUT_DB`. It used to be `min(Max corr, 40)`, so the default 20 dB could not
  cut a 30 dB resonance and left it ~3 dB proud.

### The loop must not integrate without a drive

`SpectralController.update(..., drive_active)` — `drive_active` is a **required**
argument, and `MainWindow` passes `drive_running and not is_paused`. When it is False
the error is still computed and returned, but neither loop moves.

With the loop enabled and no audio playing, the measurement is the sensor's noise
floor, ~50 dB under demand. The servo used to run regardless and walked the level
−20 → −14 → −8 → −2 → 0 dB in four updates; the next Drive On started at full scale
with 31.9 % of samples clipping. The slider still said −20.

### External drive changes restart the control window

"One update per fresh window" is not enough on its own: the window must also have been
measured **entirely under the drive it is being compared with**. Any change to the
drive that the loop did not make itself — a sequence step, Drive On, the slider, Reset,
loading a response, a profile edit — calls `MeasurementStream.restart_control_window()`.
The next control window then starts `CTRL_SETTLE_S` after the change, and a window
already taken stops counting as a control window (`is_control_window` is re-checked when
its PSD arrives, because the drive can change while Welch is running).

Without it, found in `tests/test_closed_loop.py`: a −6 → 0 dB step put +6 dB on the DAC
correctly, then the next update read a window of mostly pre-step data against the new
demand, saw a 5 dB shortfall, and added it again — a +5 dB overshoot, clipping, and
three updates to settle. Drive On mid-stream is the same fault with a 40 dB "shortfall".

The level servo's own changes do **not** restart the window; that would stretch the loop
period. `_apply_level()` is shared, so every caller except the servo emits
`drive_changed`.

### Recovery

`CTRL_RECOVERY_S = 2.0` — one control update, the floor set by `CTRL_WINDOW_S`
(the loop cannot react faster than it can measure).

The response is asymmetric. **Growing** `|correction|` uses the user's `loop_gain` and
the `CTRL_MAX_STEP_DB` slew limit, because that is the direction where stability is at
stake. **Shrinking** it is a return to neutral — the worst case is landing at zero
correction, i.e. driving the raw profile — so it runs at **unity gain** and may unwind
a full rail in one update, with a guard that lands exactly on zero rather than
flinging the bin out the other side. Recovery is therefore 2 s at *every* gain
setting, where a symmetric law took 14–28 s.
- The displayed PSD is an EMA (`PSD_DISPLAY_ALPHA`) of the raw windows, purely for a
  smooth trace. **Never feed the EMA back into the loop** — that reintroduces exactly
  the lag this design removes. Control reads the raw `results` argument.
- `max_correction_db = 20 dB` default, range 3–200 dB — the **boost** ceiling. It is a
  ceiling, not a target; the loop uses only what it needs. The error label shows
  `sat N%`, the share of in-band bins on *either* rail, and turns red above 5%.
- The loop only acts **inside the profile band** (`bp[0]` … `bp[-1]`); error, clamp
  and the error-RMS readout are all confined there, because the drive synthesises
  nothing outside it
- Correction falls back to the saved speaker response (not to flat) on test start
  and on Reset
- Control is active whenever the Enable checkbox is ticked, control channels are
  selected **and the drive is playing** — it does **not** require a running test
  sequence, but it does require a drive (see "The loop must not integrate without a
  drive")

### Speaker response persistence

The converged correction is the inverse transfer function of the whole drive chain
(amp + speaker/shaker + fixture). It is rig-specific but stable between runs, so it
is persisted rather than relearned every time.

- **Save** writes `speaker_response.json` next to the script: `freqs_hz` + `corr_db`
  on the Welch grid. Do this once the error has converged.
- The file is **auto-loaded at startup** as the controller's base response
  (`SpectralController.base_db`), and the applied correction (`corr_db`) starts as a
  copy of it, so the loop begins from the known response instead of flat.
- On load the curve is re-gridded with `np.interp` in log-frequency and **tapered to
  0 dB outside the saved span** — never edge-held, which would reintroduce the
  out-of-band runaway.
- `audio_started` fires when Drive On creates the worker, so a loaded response is
  applied immediately — otherwise it would be ignored until the next Welch cycle,
  and never applied at all with the loop disabled.
- **A loaded response is checked against the loop's own clamp**
  (`−min(max_corr, CTRL_MAX_CUT_DB)` … `+max_corr`, at the Max-corr setting in force).
  If more than `RESPONSE_MAX_CLAMPED_FRAC` of the curve is outside it, the file is
  **refused** (`ResponseError`) and the loop starts flat; otherwise it is clipped to the
  clamp. Loading can therefore never apply more than the loop itself would be allowed
  to. A refused file is reported in the status bar at startup and in a dialog on Load.
  Malformed files (bad JSON, NaN, unequal lengths, non-increasing frequencies) are
  refused the same way.
- The file is applied with the loop **disabled** too, which is why this matters: a
  wound-up file from before the asymmetric clamp (−22…+120 dB, in-band mean 103 dB) was
  being loaded unclamped at every start, putting 94 % of drive power in 1–2 kHz.
- `speaker_response.json` is rig-specific and is in `.gitignore`.

**What `max_correction_db` actually costs.** The clamp sets the spread between the
loudest and quietest part of the drive. The block is normalised to unit RMS, so a bin
boosted by *X* dB pulls every other bin down. A bin pinned near the top of the range
takes almost all the power and starves the part of the band that was tracking
correctly — the in-band version of the out-of-band runaway documented above. Very
wide spreads also approach the DAC noise floor (~96 dB at 16-bit, ~120 dB at 24-bit)
and push the voice coil past excursion/thermal limits at frequencies it cannot
reproduce, giving distortion rather than motion.

The clamp is still a ceiling rather than a target, so a high setting is harmless while
the loop does not need it — and since the limit-cycle fix the correction no longer
winds to the rail spuriously, only when there is a genuine persistent shortfall.
That is what the `sat %` readout is for: persistently non-zero saturation means those
frequencies are past what the rig can deliver, and **narrowing the profile band will
converge better than raising the clamp further**.

**Error RMS** is the level error combined with the RMS of the *frequency-smoothed*
shape error — the error the loop actually acts on. On a perfect signal it reads ~0.7 dB.
The raw per-bin RMS it replaced read 2.87 dB on a perfect signal, against a green limit
of 3 dB, so it could not show convergence. With the loop enabled but the drive off or
paused, the label reads `Error RMS: — (drive off)` / `(paused)`.

**Error display colours:**
- Green < 3 dB RMS (converged)
- Yellow 3–10 dB (converging)
- Red > 10 dB (not converged / open loop)

### Known-good FSR / sensitivity mapping
| FSR setting | Sensitivity (LSB/g) |
|-------------|-------------------|
| ±16 g | 2048 |
| ±8 g | 4096 |
| ±4 g | 8192 |
| ±2 g | 16384 |

Must match `ICM_ACCEL_CONFIG` in `firmware/main.c`. Reconnect after changing.

### Dependencies
```
pyqtgraph>=0.13
PyQt5>=5.15
pyserial>=3.5
numpy>=1.23
scipy>=1.10
sounddevice>=0.4
PyOpenGL>=3.1
```

---

## Common failure modes

| Symptom | Cause | Fix |
|---------|-------|-----|
| All axes read 0x8000 | Gyro LN not enabled (PLL off) | `PWR_MGMT0 = 0x0F`, 50 ms wait |
| PSD x-axis wrong | Double log10 from setLogMode | Never call `setLogMode` on spec plot |
| FS shows ~750 kHz | Wall-clock serial timing | Use constant `SAMPLE_RATE`, don't measure |
| Audio level not changing between steps (open loop) | Step gain was applied inside the unit-RMS block and normalised straight back out | Step gain is applied as output level: `output_dbfs(gain_db)` via `_apply_level()` |
| Control loop does nothing | `_running` guard required test sequence to be active | Loop now fires on every Welch result whenever Enable is checked, regardless of test state |
| Can't tell if loop is working | No drive-curve overlay | Green dotted "Drive" curve on spectrum plot shows `demand × H_corr` — divergence from Target shows active correction |
| editingFinished + setText infinite loop | Qt5 double-fire bug | Use only `returnPressed`, not `editingFinished` |
| Loop pinned at max correction, error RMS stuck ~30 dB, measured PSD far below target everywhere | Drive synthesised out to Nyquist (`psd_interp_loglog` clamps, doesn't roll off) **and** correction held its saturated edge value past 4 kHz. Out-of-band power dominated the unit-RMS normalisation and crushed the in-band level by ≈42 dB — a positive-feedback runaway | Band-limit synthesis to the profile band; taper correction to 0 dB outside the control band; confine the loop's error/clamp to the profile band |
| Control loop oscillates, spiky drive curve | Loop gain too high for a pure integral controller with noisy Welch bins | Error smoothed with `gaussian_filter1d(sigma=5)` before integrating |
| Measured Grms does not follow sequence steps; Level reads the ceiling; `DRIVE AT LIMIT` | The rig needs more level than the PC may send (Max drive level, default −12 dBFS) | Turn the amplifier up by at least the shortfall shown, or lower / narrow the profile. Not a software fault |
| Level `+0.0 dB`, `CLIP`, resonance stays hot though its correction is on the cut rail, `sat` | Max drive level raised to 0 dBFS: a third of samples clip and the distortion feeds the resonance | Put Max drive level back to −12 dBFS and add amplifier gain |
| Spike at the top edge of the correction curve | Top few Hz of the band had no drive (last drive bin centre is below the edge) | Synthesis covers the band to its edges, half a drive bin of slack |
| A resonance stays ~3 dB proud with Max corr at 20 dB | Cut rail was tied to Max corr | Cut rail is always 40 dB |
| Level creeps to 0 dB with the drive off; full-scale burst on Drive On | Level servo integrated ambient noise | `update(..., drive_active=False)` integrates nothing; Drive On re-seeds from the slider |
| Shaker gets louder after Stop / at test end | Drive kept running while demand reverted to 0 dB | Stop and completion stop the drive |
| +5 dB overshoot and CLIP right after a sequence step or Drive On | Control window held data from before the change, so the change was "corrected" a second time | `restart_control_window()` on every external drive change |
| Spectrum, status and loop frozen after reconnect, for as long as the last session ran | Sample counter reset but the Welch gates were not | `MeasurementStream.reset()` clears all of them, on connect |
| IN SPEC readout always ABORT | Verdict taken on single 3-average windows (2.9 dB/bin scatter) | Verdict on the average of N control windows; Settings tab |
| Rig settles ~0.8 dB above demand with error reading zero | Level servo nulled the mean of dB errors, which is biased low | Level error from linear in-band power |
| Meas Grms √2 / √3 above demand with 2 / 3 control channels | Readout summed channels, loop used their mean | One mean PSD for readout, status and loop; band-limited Grms |
| `speaker_response.json … NOT loaded` in the status bar | Saved curve is mostly outside the clamp — a wound-up integrator, not a response | Relearn with the loop, then Save |
| Every frequency reads 1.3 % low; top edge of the profile band saturates | Stream arriving at the sensor's 8107 Hz but treated as 8000 Hz — status line not reaching `set_input_rate` (old host, or parser dropped it) | Toolbar must read `FS: 8000 Hz (sensor … Hz, resampled)` within 2 s of connecting |
| Toolbar stays at `FS: 8000 Hz (configured)` | Firmware older than 2026-10-08 (no status line) | Works, but that firmware skips ~107 samples/s and has the default filters — flash the current one |
| Spectral lines have sidebands ~108 Hz apart | Firmware reading on an MCU timer instead of once per DRDY | Current firmware; check `tools/hw_check.py info` says `drdy-polled` |
| Measured PSD droops toward 2 kHz on a rig known to be flat | Sensor filters at reset defaults (−9 dB at 1.4–2 kHz) | `tools/hw_check.py info` must show `now=01,0D,7E,80,3F` |
| 4 flashes on the LED | Accel filter registers did not read back | SPI integrity; bank select not returned to 0 |
| Host shows `Drops: 0` across a USB stall | `seq` was assigned at transmit, after the ring dropped samples | `seq` stamped at acquisition on Core 0 |
| `PICO_BOARD` not found at configure | Board is named `pimoroni_pico_plus2_rp2350` in SDK 2.2.0 | Use that name |
| Sustained Grms limit cycle (~14 s period, rail-to-rail drive), unaffected by `loop_gain` | Loop updated at ~100 Hz against a 10 s moving-average measurement — ~1000 updates per measurement refresh, so every gain saturated to effective 1.0 | Welch over the most recent `CTRL_SAMPLES` (2 s) only, launched once per that many *new* samples → each update sees a fresh non-overlapping window. Also cut Welch CPU 200× |
