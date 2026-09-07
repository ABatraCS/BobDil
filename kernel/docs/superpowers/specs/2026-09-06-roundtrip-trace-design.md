# Round-trip span trace

A tool that answers "where did the time go, from the driver's hand to the
driver's hand?" -- input sample, through the step, out to the wheel.

It exists because `docs/architecture.md` 5.6 states a latency budget
(~3-6 ms round trip, of which ~1 ms is ours) that nothing in this repository
measures. `bench` measures step time and `metrics.rs` publishes it, but step
time is one span in the middle of the chain. A step that costs 200 us inside a
loop that hands the wheel a 5 ms old torque is a rig that feels rubber-banded,
and today nothing here would say so.

## What it is not

Not a sampled CPU profiler. `perf` answers "which Rust function burns the
budget"; this answers "which *stage of the round trip* burns the budget",
which is a different question and the one 5.6 poses. The two are complementary
and this is the one that spans processes.

## Scope

Covered:

- **input age** -- the consumed sample's stamp to the start of the step that
  consumed it, including whether the sample was *re-used* (the loop ran, the
  driver's input was stale)
- **the step, broken into five spans** -- read, shape, plant, ffb, publish
- **publish to pickup** -- the step thread publishing a torque to the device
  thread taking it
- **apply** -- the cost of handing that torque to the device

Not covered, deliberately, and left with a documented seam:

- **the UI leg** -- state frame visible in `/dev/shm` to pixels the driver
  sees. The Godot view opens the state segment `READ` and physically cannot
  write (`view/scripts/state_link.gd`), so stamping it means giving the view a
  write path to something. That is a decision worth making on its own, and at
  60 Hz the view's frame pacing (~16 ms) will dominate every number here when
  it does land -- which is the reason to keep it a separate, visible change
  rather than a footnote in this one.
- **USB and wheel firmware** -- roughly half the budget by 5.6's own estimate,
  and not observable from this side of the kernel at all.

## Decisions

### Off by default, one branch when off

Five extra `monotonic_ns()` reads and a ring push per step, ~125 ns against a
1 ms budget. That is 0.0125% and well inside the p99.9 < 500 us gate's noise,
but it is still the critical path, so it is behind `--trace` and costs one
predictable branch on a `bool` when off.

### A lossless ring, not a seqlock

The three existing latest-value channels are seqlocks because they want the
newest value and a backlog would be wrong. A trace wants the opposite: the
whole point is the tail, and a seqlock drops samples by design -- it would
systematically lose exactly the slow steps being hunted. So the trace goes
through `transport/spsc_ring.rs` and a drain thread, mirroring
`telemetry/recorder.rs`.

Two rings, because `spsc_ring` is single-producer and there are two producer
threads (step and device). One drain thread reads both.

### Not fields on `vehicle_state`

Riding in the existing `.bdt` recording would be free, and it is unsafe.
`replay` proves two runs produce bit-identical states; wall-clock stamps are
nondeterministic by construction. Putting them in the compared struct would
break the determinism guarantee `replay-check` exists to enforce.

### In the schema, at the cost of one `layout_hash` bump

The records cross a Rust->Python boundary, so AGENTS.md puts them in
`schema/bobdil_signals.yaml`. `_layout_hash` fingerprints every frame
together, so adding them moves the global hash once: existing `.bdt`
recordings must be re-recorded and the view and C header rebuilt. Accepted
deliberately rather than paid for with a second hand-maintained binary format.

### `shm_name` becomes optional

`schema.py` requires an `shm_name` on every frame. These two are ring and file
records, not shared-memory segments, and giving them a fake segment name would
be a lie in the file that is supposed to be the single source of truth. One
guard in `schema.py` and one in each of the five emitters. `_layout_hash` does
not hash `shm_name`, so this change alone does not move the fingerprint.

### The join key already exists

`ffb_command.host_time_ns` is the step thread's publish stamp, and
`io/hid.rs` already reads `monotonic_ns()` before picking the command up. The
device record carries that stamp and the tool joins on it. No new field on
`ffb_command`, and the pickup leg costs no new clock read.

### A stale command is a fault, not a slow span

`trace_device.flags` carries the watchdog verdict. Without it a rejected or
stale command would enter the percentiles as a large latency rather than
appear as a broken leg -- the tool would average a safety condition into a
timing number and quietly misreport it.

## Records

Both frames in `schema/bobdil_signals.yaml`. The schema's `types` map offers
`f64`/`u64`/`i64` only, so stamps are absolute `u64` nanoseconds rather than
packed deltas. At 1 kHz that is ~120 KB/s, which is not worth optimising.

`trace_step`, one per step, pushed by the step thread:

| field | why |
| --- | --- |
| `step_index` | joins to `vehicle_state`, so a slow step can be found in telemetry |
| `input_host_time_ns` | the consumed sample's stamp; the input-age leg |
| `input_sample_index` | detects a re-used sample: loop ran, input was stale |
| `t_step_start` | |
| `t_after_read` | bounds the seqlock read |
| `t_after_shape` | bounds `input_shaper` |
| `t_after_plant` | bounds `plant.step` -- the span Phase 0 cares about |
| `t_after_ffb` | bounds the conditioning chain |
| `t_after_publish` | bounds both seqlock publishes |

`trace_device`, one per HID iteration, pushed by the device thread:

| field | why |
| --- | --- |
| `command_host_time_ns` | the join key: the step's publish stamp |
| `t_pickup` | already computed in `hid.rs`; free |
| `t_after_apply` | the only new clock read on this thread |
| `sample_index` | ties outgoing torque back to the input that caused it |
| `flags` | the watchdog verdict; see above |

## Transport and file

A `.bdtrace` file with its own magic and version in its header, written by a
drain thread with no deadline. It refuses loudly on a magic, version or
`layout_hash` mismatch, exactly as `recording.py` does -- a trace parsed with
the wrong meaning is worse than no trace.

## Output

`tools/trace/`, sibling to `tools/rt_bench/`:

- **Chrome Trace Event JSON** -- the artifact. Perfetto, `chrome://tracing`
  and speedscope all render it as a zoomable waterfall with their own
  summaries, so there is no hand-rolled SVG to maintain and no dependency to
  add.
- **A terminal summary** -- p50/p99/p99.9 per span, plus counts of stale-input
  and stale-command steps, so the make target says something useful with no
  browser.

## Testing

Per the AGENTS.md split these numbers are **validation** group: they measure
this box and they do not travel. Nothing here may be quoted as a property of
the model.

The tests therefore assert plumbing, not timings:

- `--trace` off pushes nothing into either ring
- spans within a step are monotonic and nest inside the step bounds
- a synthetic `.bdtrace` round-trips through the Python reader
- a wrong magic, version or `layout_hash` is refused rather than parsed
- a stale-command record is counted as a fault and kept out of the percentiles
- `codegen-check` passes with `shm_name` optional and the fingerprint unmoved
  by that change alone
