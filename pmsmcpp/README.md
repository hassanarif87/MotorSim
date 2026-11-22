# PMSM C++ simulation

From the repository root, run the following with Bazelisk or Bazel 8.4.0.

```sh
cd pmsmcpp
bazel build //:pmsm_sim
bazel test //...
bazel run //:pmsm_sim
```

The example reads `pmsm_config.json` and saves `pmsm_results.csv` in this
directory. Pass an output path and, optionally, a configuration path after
`--`. Relative paths are resolved from the Bazel workspace root.

```sh
bazel run //:pmsm_sim -- /tmp/my_run.csv
bazel run //:pmsm_sim -- /tmp/my_run.csv my_config.json
```

## Guides

- [Assemble a model from JSON](README.assembly.md#assemble-a-model-from-json)
- [Simulation architecture](README.architecture.md)

## Add a new C++ block type

This example adds a periodic current gain that can sit between the speed
controller and current PI. Add the following inside the
`pmsm::drive_blocks` namespace in [`blocks/drive_blocks.h`](blocks/drive_blocks.h).
The factory expects a constructor taking `const BlockSpec&` and public
members named `inputs` and `outputs`. Boost.Describe exposes the port names
and their types to the graph builder.

```cpp
struct CurrentGainInputs {
    InputPort<Current> current;
};

struct CurrentGainOutputs {
    Current current;
};

BOOST_DESCRIBE_STRUCT(CurrentGainInputs, (), (current))
BOOST_DESCRIBE_STRUCT(CurrentGainOutputs, (), (current))

class CurrentGain {
public:
    explicit CurrentGain(const BlockSpec& spec)
        : gain_(config_detail::number(spec.parameters, "gain")) {}

    void tick() {
        outputs.current = gain_ * inputs.current.read();
    }

    CurrentGainInputs inputs;
    CurrentGainOutputs outputs;

private:
    double gain_;
};
```

Add `X("current_gain", CurrentGain, periodic)` to the
`PMSM_BLOCK_TYPES` list in [`drive.h`](drive.h). Append the entry as the
last line of the macro and add a trailing backslash to the previous line.
The factory binds `tick()` for periodic blocks. A continuous block exposes
`advance(SimulationExecutor::Duration)` and owns its integration state.
A constant block publishes its outputs in its constructor and needs no clock.

Start with a fresh copy of the default config and add this instance to
`blocks`.

```json
{
  "name": "reference_gain",
  "type": "current_gain",
  "rate_hz": 20000,
  "priority": 2,
  "parameters": {"gain": 0.5}
}
```

Replace `["current_pi.iq_ref", "speed_control.iq_ref"]` with these two routes.

```json
[
  ["reference_gain.current", "speed_control.iq_ref"],
  ["current_pi.iq_ref", "reference_gain.current"]
]
```

The gain runs before the PI when their ticks coincide. With the default
3 A startup request from the speed controller, the logged PI reference
starts at 1.5 A. Rebuild and check the run with the commands below.

```sh
bazel test //...
bazel run //:pmsm_sim -- /tmp/current_gain.csv my_config.json
```

The drive constructs blocks at stable addresses and validates the graph
before opening the CSV. During the run, callbacks read bound ports
directly. Each block keeps its state in its own instance.

## Default schedule

The example starts the PMSM from rest and advances it for 0.4 s using RK4
with a maximum step of 10 µs. The executor shortens steps to land on clock
events. The speed loop ticks every 1 ms, feedback and current PI every 50 µs,
SVPWM and inverter every 50 µs, and logging every 100 µs. The PI advances its
integrators once per tick. The inverter holds its phase voltages between
updates. The CSV records both endpoints and the latest published port values.
