# PMSM model assembly

Run the commands in this guide from `pmsmcpp`. See the [main README](README.md) for build instructions and the [architecture README](README.architecture.md) for how the simulator runs the assembled model.

## Assemble a model from JSON

Copy [`pmsm_config.json`](pmsm_config.json) to `my_config.json` and edit the
following sections.

| Section | What it controls |
| --- | --- |
| `simulation` | Base tick frequency (`tick_rate_hz`), simulation length in seconds (`duration_s`), and CSV rate (`log_rate_hz`) |
| `blocks` | Instances, their types, parameters, and digital clocks |
| `connections` | Signal routes as `[destination, source]` pairs |
| `log_columns` | CSV columns as `[header, port]` pairs |

Each instance has a unique `name` without dots and a registered `type`. Put
its settings in `parameters`. Periodic blocks require `rate_hz` and accept
an optional `priority` (default 0). Lower priorities run first at coincident
ticks, with JSON order breaking ties. Frequencies must be positive integers
that divide 1,000,000,000 exactly. Each block and CSV rate must also divide
`tick_rate_hz` exactly. For example, 20,000 Hz runs every 50 ticks at a
base frequency of 1,000,000 Hz. Older configs can set the tick interval with
`step_s` and the duration with `steps`. Their block and logging periods must
span whole simulation ticks. The executor supports one continuous block
and up to 15 periodic blocks, reserving one slot for CSV logging.

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

### Set the motor load torque

Set `value_nm` in the parameters of the block named `load`. The
`["pmsm.load_torque", "load.value"]` connection applies that constant
torque in newton-metres to the motor. For an older configuration, move
the motor’s `load_torque_nm` setting to the new block’s `value_nm` and
add the connection.

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
3. Run `bazel run //:pmsm_sim -- my_config.json /tmp/fixed_current.csv`.
   The default `iq_ref_a` column reads the PI input and should report 2 A.

To create a block type that you can instantiate in JSON, see [Add a new C++ block type](README.md#add-a-new-c-block-type).
