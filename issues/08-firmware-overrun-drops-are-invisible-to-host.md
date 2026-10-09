# Firmware: ring overruns are invisible to the host drop counter

**Severity:** Medium (silent data loss; stale data on every connect)
**Location:** `firmware/main.c:120-122` (`ring_push`), `297-298` and `300-322` (`core1_main`)
**Status:** Confirmed by reading only. Not run on hardware.

## Problem

`ring_push` returns silently when the ring is full. The frame sequence number is assigned on Core 1 after `ring_pop`, and increments once per transmitted frame, so the stream is always contiguous. The host counts drops from gaps in `seq`, so it never sees an overrun.

On every boot Core 1 waits in `while (!stdio_usb_connected())` while Core 0 is already sampling. The 4096-entry ring fills in 512 ms and everything after is dropped. On connect the host receives 512 ms of stale boot-time data, then an unreported gap, and shows "Drops: 0". Any later USB stall behaves the same way.

## Suggested fix

- Stamp `seq` on Core 0 at acquisition and store it in `sample_t`, so a dropped sample leaves a gap the host already detects.
- Or count overruns on Core 0 and transmit the counter.
- Flush the ring (`ring_rd = ring_wr`) when the USB connection is first seen.

Changing the frame layout changes the wire protocol; update `FRAME_BYTES`, the host parser and CLAUDE.md together.

## Acceptance

Stalling the host reader for 1 s produces a non-zero drop count of about 8000 minus the ring size.

## Resolution (2026-10-08)

**Fixed and verified on the hardware.** Firmware `2026-10-08`.

`seq` is stamped on Core 0 for every sample read and carried through the ring. Core 1 discards the ring when the host first connects. The frame layout is unchanged.

Measured on the rig (`python tools/hw_check.py`), old firmware (May 2026 build) against new:

| | Old | New |
|---|---|---|
| First sequence number on the first connection after boot | 0 | 44835 (5.5 s of sampling had already happened) |
| Frames received beyond what the elapsed time accounts for, first capture after boot | +4073 (the stale ring; 4115 on a second boot) | +371, inside the capture's ±400 timing tolerance |
| 3 s host stall: drops reported | 22772 | 23092 |
| 3 s host stall: samples lost without being numbered | −5 ± 18 | −3 ± 17 |
| 1 s host stall: drops reported / unreported | 6772 / −3 ± 18 | 6836 / −6 ± 17 |

Two things the measurements showed that the ticket did not expect:

- On the old firmware a host stall did **not** hide any loss. The Pico SDK's USB write gives up after 500 ms, and the ring holds 512 ms, so the ring never quite overflowed; the samples were lost in the USB write, after they had been numbered. The stale 512 ms at first connection was real and is reproduced above.
- The fix is still the right one: it no longer depends on that 12 ms of luck, and the sample loop now also numbers any sample the sensor produced that Core 0 was too late to read.

Tests: `tests/test_serial.py::test_firmware_overrun_shows_up_as_drops`, `::test_drop_count_survives_sequence_wraparound` (host side); hardware results above.
