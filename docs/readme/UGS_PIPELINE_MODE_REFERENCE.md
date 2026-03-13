# UGS Pipeline Mode vs Single-Step Mode — Reference Document

**Purpose**: Permanent reference so we never have to re-research this.  
**Source**: Official GRBL v1.1 Interface wiki (https://github.com/gnea/grbl/wiki/Grbl-v1.1-Interface)  
**Date written**: March 13, 2026

---

## 1. The Two Streaming Modes

### Single-Step (Send-Response) Mode
- UGS sends **one line**, then **waits** for `ok` or `error:` before sending the next line.
- No window, no counting. One round-trip per G-code line.
- Slowest mode but most robust.
- Never stalls due to `ok` deficits — it simply won't send the next line until the previous `ok` arrives.
- **Why it works even with our bugs**: `rxBuffer` only ever holds one line at a time. Any wipe-on-completion is harmless because the tail is always empty.

### Pipeline (Character-Counting) Mode
- UGS maintains a **running count of bytes in flight** (bytes sent but not yet acknowledged by `ok`).
- UGS can send **multiple lines ahead** as long as the total unacknowledged byte count stays below the **RX buffer size window**.
- When an `ok` arrives, UGS subtracts the byte count of the corresponding line from its running total and may send more lines.
- This fills the UART RX ring and GRBL's lookahead planner simultaneously — maximum throughput.
- **Why it stalls with our bugs**: Multiple lines can be pre-loaded into `rxBuffer` simultaneously. If the firmware wipes `rxBuffer` after processing only the first line, all subsequent lines are destroyed with no `ok` sent. UGS's byte-count window fills with unacknowledged bytes → UGS stops sending → machine drains its queued moves → sits `<Idle>` with UGS frozen waiting for `ok`s that will never come.

---

## 2. The `[OPT:]` Field — The Critical Handshake

When UGS connects, it sends `$I` and reads back:

```
[VER:1.1h.20251102]
[OPT:VHM,35,127,4]
ok
```

The `[OPT:]` format is: `[OPT:option_flags, blockBufferSize, rxBufferSize, axes]`

| Field | Our value | Meaning |
|-------|-----------|---------|
| `VHM` | flags | V=variable spindle, H=homing single-axis, M=mist coolant |
| `35` | blockBufferSize | Lookahead planner queue depth (trajectory slots) |
| `127` | **rxBufferSize** | **This is UGS's pipeline window size in bytes** |
| `4` | axes | Number of axes |

### Why `rxBufferSize` matters critically

**UGS uses the `rxBufferSize` value as the maximum number of unacknowledged bytes it will allow in flight at any time.**

If we advertise `127`, UGS will never have more than 127 bytes of G-code in flight simultaneously. A typical G-code line is 10–20 bytes, so with 127 bytes UGS can pipeline at most 6–12 lines ahead. This significantly limits lookahead and can cause motion stutter on high-speed short-segment paths.

If we advertise `512` (our actual UART3 RX ring buffer size), UGS will pipeline up to 512 bytes — ~25–50 lines — providing much better planner fill.

**Our firmware has this hardcoded in `srcs/settings/settings.c`:**
```c
const char build_info[] = "[VER:1.1h.20251102]\r\n[OPT:VHM,35,127,4]\r\nok\r\n";
```

The `127` should match the actual `UART3_READ_BUFFER_SIZE` defined in `srcs/config/default/peripheral/uart/plib_uart3.c`.

---

## 3. How the Character-Counting Window Works (Concrete Example)

Suppose UGS has these G-code lines queued:
```
G1X60Y0       (10 bytes incl. \n)
G1X60Y60      (10 bytes)
G1X0Y60       (9 bytes)
G1X0Y0        (8 bytes)
G1F6000       (8 bytes)
```

With a 127-byte window:
1. UGS sends lines 1-10 (up to 127 bytes total) without waiting.
2. Firmware receives `ok` for line 1 → UGS subtracts 10 from its counter (now 117 in flight).
3. UGS sends next line immediately because 117 + next_line_length < 127.
4. Process repeats — pipeline is kept full.

**If a single `ok` is missing:** The counter never goes down enough. Eventually the window fills and UGS stops sending. If the machine is still running from its lookahead buffer it will eventually drain and go `<Idle>`. UGS sits frozen waiting for the missing `ok`s.

---

## 4. The Root Bug We Fixed (March 10, 2026)

### The bug

Three parser states in `gcode_parser.c` did this after processing the first line they saw:

```c
nBytesRead = 0;
memset(rxBuffer, 0, sizeof(rxBuffer));  // WIPES THE ENTIRE BUFFER
gcodeData.state = GCODE_STATE_IDLE;
```

**In pipeline mode**: `GCODE_STATE_IDLE` reads ALL available UART ring bytes into `rxBuffer` in one pass each main-loop iteration. So `rxBuffer` frequently contains 2, 3, or more complete G-code lines simultaneously.

**Effect**: Only the first line was processed and got an `ok`. All subsequent lines in `rxBuffer` were silently destroyed. No `ok` ever sent for them. UGS's byte window accumulated unacknowledged bytes → stall.

The three broken states:
- `GCODE_STATE_QUERY_CHARS` (handles `$` commands)
- `GCODE_STATE_GCODE_COMMAND` (handles `G`/`M`/`F`/`S`/`T` commands)
- `GCODE_STATE_ERROR` (error handler)

### The fix

Replaced the wipe with the `memmove` tail-preservation pattern that `GCODE_STATE_IDLE` already used correctly:

```c
// After processing command at position [0..cmd_end]:
uint32_t skip_pos = cmd_end + 1u;   // skip past the NUL terminator
while (skip_pos < nBytesRead && (rxBuffer[skip_pos] == '\r' || rxBuffer[skip_pos] == '\n'))
    skip_pos++;  // also skip trailing CR/LF of this command
uint32_t remaining_bytes = nBytesRead - skip_pos;
if (remaining_bytes > 0) {
    memmove(rxBuffer, &rxBuffer[skip_pos], remaining_bytes);  // slide next command to front
    nBytesRead = remaining_bytes;
    rxBuffer[nBytesRead] = '\0';
} else {
    nBytesRead = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));  // buffer really is empty
}
```

### Why single-step mode was immune

In single-step mode UGS sends one line and waits. By the time IDLE reads from the UART ring, there is exactly one line in `rxBuffer`. The tail after that line is always empty. The wipe was harmless — there was nothing behind it to destroy.

---

## 5. The Remaining Issue After the Fix

The `memmove` fix prevents `ok` destruction. However there is still an `ok` deficit visible in the UGS logs (machine completes motion, goes `<Idle>`, but UGS never prints `*** Finished sending file`).

**Analysis of `tests/07_complex_long_run_fast.gcode`:**
- Total lines in file: 199
- Non-blank/non-comment lines (each needing one `ok`): 147
- UGS tracks `ok` count vs lines sent — it only says "Finished" when its counter reaches zero

**Current advertised RX buffer**: `127` bytes → limited pipeline depth

The remaining investigation is whether the deficit is:
1. A remaining code path in the parser that still wipes the buffer (e.g. the G92/blank line handling)
2. The `okPendingCount` deferred-ok path losing credits under some edge case
3. Some commands (blank lines, comments) not generating `ok` in all paths

Use `$U` / `$U=0` before and after a run to measure `OK_ATT` vs the expected line count (199 total lines, all get `ok` in GRBL including blank lines and comments — GRBL spec says even empty lines get `ok`).

---

## 6. Quick Reference: What advertised `rxBufferSize` to use

| UART3 RX ring size | Advertise in `[OPT:]` |
|---|---|
| 128 | 127 (GRBL convention: size - 1) |
| 256 | 255 |
| 512 | 511 |
| **1024** | **1023 ← we are here** |

**GRBL spec convention**: advertise `rxBufferSize - 1` to prevent exact-fill edge cases.

Our UART3 RX ring is defined in `srcs/config/default/peripheral/uart/plib_uart3.c`:
```c
#define UART3_READ_BUFFER_SIZE   (1024U)
```

So we should advertise `1023` in `[OPT:]`, not `127`.

The fix is one line in `srcs/settings/settings.c`:
```c
// Change:
const char build_info[] = "[VER:1.1h.20251102]\r\n[OPT:VHM,35,127,4]\r\nok\r\n";
// To:
const char build_info[] = "[VER:1.1h.20251102]\r\n[OPT:VHM,35,1023,4]\r\nok\r\n";
```

This tells UGS it can keep up to 1023 bytes in flight, filling our 1024-byte RX ring and keeping the trajectory planner fully fed.

---

## 7. Key Rules — Never Forget

1. **Every line sent by UGS must get exactly one `ok`** — including blank lines and comment-only lines. GRBL spec is explicit: empty lines return `ok`.
2. **`[OPT:] rxBufferSize` = UGS pipeline window** — too small = slow pipeline; too large = UART ring overflow.
3. **`rxBuffer` is not a one-line buffer in pipeline mode** — it can hold many lines. Any code that wipes it after processing line 1 breaks pipeline.
4. **`?` status queries are real-time** — they bypass the serial buffer entirely on real GRBL (intercepted before entering ring). On our firmware they enter the ring like normal bytes but are picked off quickly in `GCODE_STATE_CONTROL_CHAR`. They never consume an `ok` credit.
5. **Settings commands (`$x=`) should NOT be sent in pipeline mode** — GRBL wiki warns that EEPROM writes disable serial RX interrupt briefly (AVR-specific) but the principle holds: don't mix settings flashes with pipelined motion.
6. **Push messages (`<...>`, `[...]`, `ALARM:`) are NOT `ok` responses** — UGS discards them from its streaming count. Only bare `ok` or `error:` count.
