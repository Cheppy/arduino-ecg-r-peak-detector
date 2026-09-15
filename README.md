# ECG R-Peak Detector

Real-time ECG acquisition and R-peak detection on an Arduino Mega 2560 + AD8232,
implemented as a **streaming (sample-by-sample) pipeline** with bounded memory.

Reference: <https://doi.org/10.32620/reks.2026.1.02>

## Hardware
- Arduino Mega 2560 (ATmega2560, 8-bit, no FPU)
- AD8232 ECG analog front-end
- Standard lead-II electrode placement

## Files
- `ecg_r_peak_detector/ecg_r_peak_detector.ino` — Arduino sketch
- `ecg_r_peak_detector/config.h` — configuration constants

## Algorithm

Every acquired sample passes through the full pipeline before the next sample is
taken. Nothing is accumulated over a frame, and no state grows with recording
length.

1. **Acquisition** — `analogRead()` on a `micros()` schedule, re-centred to
   signed counts (`value - 512`) and pushed into a circular buffer.
2. **Cascaded smoothing** — `K` identical 3-tap Hermite stages:

   `H[n] = (F[n-1] + 6·F[n] + F[n+1] + 4) / 8`

   Integer only: three additions, one multiply by 6 (two shifts and an add) and
   one right shift by 3. Producing `H[n]` needs `F[n+1]`, so each stage delays
   the stream by exactly one sample.
3. **Differentiation** — squared central difference on the smoothed stream,
   kept only on falling slopes:

   `d[m] = S[m-1] - S[m+1]`,  `der[m] = d² if d > 0, else 0`

   The difference is **not** normalised by the sampling interval, so `THRESHOLD`
   is expressed in squared ADC counts and is device-specific.
4. **Detection** — a run is opened at the first sample where
   `der[m] >= THRESHOLD` and the refractory period has expired, and stays open
   while the derivative remains above the threshold (bounded to `QRS_WIDTH`
   samples). The R apex is the maximum of the **raw** signal over
   `[runStart - QRS_WIDTH, runEnd]`: the part before the run is scanned once
   when the run opens, the rest is folded in incrementally as samples arrive, so
   tracking the whole span costs no extra buffer. When the run closes, the apex
   is reported and a `REFRACTORY_MS` blanking period suppresses further
   detections.

Total pipeline latency is `K + 1` samples (4 samples ≈ 11 ms at 360 Hz with the
default `K = 3`).

## Memory

Static footprint of the pipeline, with the default configuration
(`QRS_WIDTH = 30`, `SMOOTH_MAX_ITERATIONS = 8`):

| Item | Bytes |
|---|---|
| Raw circular buffer, `RAW_BUF_LEN = 42` × `int16_t` | 84 |
| Smoothing cascade state, 8 × (2 × `int16_t` + flag) | 40 |
| Differentiation delay line | 5 |
| Above-threshold run state | 19 |
| Indices, counters, run-time settings | 33 |
| **Total** | **181** |

Lowering `SMOOTH_MAX_ITERATIONS` to the 3 passes actually used brings this to
about 156 bytes. The Arduino `HardwareSerial` RX/TX ring buffers (128 bytes on
the Mega) and the stack are on top of this.

## Output modes

Selected at compile time with `OUTPUT_MODE` in `config.h`, and switchable at run
time with the `M` command.

### `MODE_FULL` (0) — one CSV line per sample

```
raw,smoothed,derivative_squared
```

For visualisation and threshold tuning. About 2.8 kB/s at 360 Hz; `BAUD_RATE`
of 250000 is recommended for headroom.

Sample lines are **droppable**: if the UART transmit buffer is full the line is
discarded and the `dropped` counter is incremented, so that acquisition timing
is never disturbed by the serial port.

### `MODE_METADATA` (1) — one line per detected beat

```
R,sample_index,amplitude,rr_ms
```

About 19 B/s at 75 bpm, versus 720 B/s for raw 16-bit streaming at 360 Hz — a
37-fold reduction. Peak lines are never dropped.

Both modes emit `#`-prefixed comment lines for banners and statistics.

## Output link

Independently of the output mode, the byte stream can be sent over USB, over a
Bluetooth module, or over both. The link is set by `OUTPUT_LINK` in `config.h`
and can be changed at run time with the `L` command.

| `OUTPUT_LINK` | Destination |
|---|---|
| `0` (`LINK_USB`) | USB — `Serial` |
| `1` (`LINK_BT`) | Bluetooth — `Serial1` |
| `2` (`LINK_BOTH`) | both, simultaneously |

Bluetooth support is compiled in only when `USE_BLUETOOTH` is `1`; it costs
about 160 bytes of SRAM for the `Serial1` ring buffers, so USB-only builds
should leave it at `0`. Selecting a Bluetooth link without it is a compile
error rather than a silent fallback.

Wiring on the Mega 2560: the module's `RXD` goes to `TX1` (pin 18) and its
`TXD` to `RX1` (pin 19). A 5 V `TX1` output must be divided down before
reaching the `RXD` input of a 3.3 V module. `BT_BAUD_RATE` must match the rate
the module is configured for — 115200 here, not the 9600 that HC-05 modules
ship with.

When `LINK_BOTH` is selected, a sample line is emitted only if **both** links
have room for it, so the slower link governs the drop rate. This is deliberate:
the two streams stay identical, which is what makes a side-by-side comparison
meaningful.

Commands are accepted from whichever link sends them, so the node remains
controllable over Bluetooth once the USB cable is disconnected.

## Serial commands

| Command | Effect |
|---|---|
| `T<n>` | set the squared-derivative threshold |
| `I<n>` | set the number of smoothing passes (1…`SMOOTH_MAX_ITERATIONS`); resets the pipeline |
| `M<0\|1>` | select output mode (0 = full, 1 = metadata) |
| `L<0\|1\|2>` | select output link (0 = USB, 1 = Bluetooth, 2 = both) |
| `S` | print statistics |
| `X` | reset the pipeline and all counters |

Statistics line:

```
# n=<samples> peaks=<count> overruns=<count> dropped=<count> mode=<m> link=<l> thr=<t> iter=<k>
```

`overruns` counts sampling instants missed because output or command handling
overran the sampling period — it should stay at 0 in `MODE_METADATA`.

## Configuration (`config.h`)

| Constant | Default | Meaning |
|---|---|---|
| `SAMPLE_PERIOD_US` | `2778` | Sampling period (≈360 Hz, matching MIT-BIH). A true period: the ~112 µs spent inside `analogRead()` is absorbed by the `micros()` scheduler. |
| `SMOOTH_ITERATIONS` | `3` | Smoothing passes applied at run time. |
| `SMOOTH_MAX_ITERATIONS` | `8` | Compile-time upper bound; sets the cascade state size. Each pass costs 5 bytes and one sample of latency. |
| `THRESHOLD` | `500` | Squared-derivative threshold, in squared ADC counts. **Tune per device**: raise it if false positives appear on P or Q deflections, lower it if beats are missed. |
| `QRS_WIDTH` | `30` | Width of the retrospective window searched backwards for the R apex (≈83 ms at 360 Hz). Not a minimum peak separation — that role belongs to `REFRACTORY_MS`. |
| `REFRACTORY_MS` | `200` | Blanking period after an accepted R-peak. |
| `OUTPUT_MODE` | `0` | `0` = full, `1` = metadata only. |
| `OUTPUT_LINK` | `0` | `0` = USB, `1` = Bluetooth, `2` = both. |
| `USE_BLUETOOTH` | `0` | Compile `Serial1` support in; costs ~160 bytes of SRAM. |
| `BT_BAUD_RATE` | `115200` | Bluetooth module rate; must match its configuration. |
| `BAUD_RATE` | `115200` | 250000 recommended for `MODE_FULL`. |
| `ECG_PIN` | `A0` | Analog input wired to the AD8232 `OUTPUT` pin. |
| `USE_LEAD_OFF` | `0` | Set to 1 if `LO+`/`LO-` are wired. |

## Tuning the threshold

If the threshold is set too low, a steep Q deflection can open a run of its own
*before* the R apex. That run closes again during the R upstroke, where the
derivative falls back below the threshold, so the reported apex is the maximum
of a window that ends before R — typically the P wave — and the refractory period
then blanks the real beat. The result is one detection per beat, mislocated by
several tens of milliseconds, rather than an obvious extra detection.

Verify with `MODE_FULL` that `derivative_squared` clears the threshold only on
the R→S descent. On a synthetic 40-beat record the detector localises every apex
exactly once the threshold is above the Q deflection; below that it still reports
40 beats, but with RR intervals scattered by ±30 samples.
