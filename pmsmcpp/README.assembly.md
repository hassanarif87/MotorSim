# PMSM model assembly

Run the commands in this guide from `pmsmcpp`. See the [main README](README.md) for build instructions and the [architecture README](README.architecture.md) for how the simulator runs the assembled model.

## Assemble a model from JSON

Copy [`pmsm_config.json`](pmsm_config.json) to `my_config.json` and edit the
following sections.

| Section | What it controls |
| --- | --- |
| `simulation` | Maximum RK4 step in seconds (`step_s`), simulation length (`steps × step_s`), and CSV rate (`log_rate_hz`) |
| `blocks` | Instances, their types, parameters, and digital clocks |
| `connections` | Signal routes as `[destination, source]` pairs |
| `log_columns` | CSV columns as `[header, port]` pairs |

Each instance has a unique `name` without dots and a registered `type`. Put
its settings in `parameters`. Periodic blocks require `rate_hz` and accept
an optional `priority` (default 0). Lower priorities run first at coincident
ticks, with JSON order breaking ties. The rate must be a positive integer
that divides 1,000,000,000 exactly. The executor supports one continuous
block and up to 15 periodic blocks, reserving one slot for CSV logging.

A connection names ports as `instance.port`. For example,
`["pmsm.va", "inverter.va"]` feeds the inverter's phase-A voltage into the
motor. Each input needs one source with the same signal type, and one
output can feed several inputs. A block reads the last value its source
published, so set priorities to put a producer before its consumer when
both tick at the same time.

The `blocks` array is required. Omitted `connections` or `log_columns`
use the workspace defaults. When you change the topology, supply the
complete connection list. Log columns can refer to numeric inputs or
outputs. The logger adds `time_s` automatically.

### Replace the speed loop with a fixed current reference

1. Remove the instance named `speed_control` from `blocks` and add the
   following instance.

   ```json
   {
     "name": "reference",
     "type": "constant_current",
     "parameters": {"value_a": 2.0}
   }
   ```

2. In `connections`, remove `["speed_control.omega", "pmsm.omega"]` and
   replace the route into the current PI with
   `["current_pi.iq_ref", "reference.value"]`.
3. Run `bazel run //:pmsm_sim -- /tmp/fixed_current.csv my_config.json`.
   The default `iq_ref_a` column reads the PI input and should report 2 A.

To create a block type that you can instantiate in JSON, see [Add a new C++ block type](README.md#add-a-new-c-block-type).
