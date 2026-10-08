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

## Resolution (2026-10-07)

**Fixed in code and built. Not flashed or run on hardware.**

`seq` is stamped at acquisition on Core 0 and carried through the ring; the ring is discarded when the host first connects. Frame layout is unchanged. Builds clean for `pimoroni_pico_plus2_rp2350`. Still to do at the rig: flash it and confirm that stalling the host produces a non-zero drop count.

Tests: Host side only: `test_serial.py::test_firmware_overrun_shows_up_as_drops`, `::test_drop_count_survives_sequence_wraparound`
