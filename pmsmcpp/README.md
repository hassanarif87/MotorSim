# PMSM C++ simulation

The simulator builds with GCC 16.2 in C++26 mode. The graph targets use
`-freflection`, and Bazel selects `gcc-16`
from `PATH`. From the repository root, run the following with Bazelisk or
Bazel 8.4.0.

```sh
cd pmsmcpp
bazel build //:pmsm_sim
bazel test //test/...
bazel run //:pmsm_sim -- pmsm_config.json
```

Pass the JSON configuration path first. The optional second argument sets the
CSV output path, which defaults to `pmsm_results.csv`. Relative paths are resolved
from the Bazel workspace root.

```sh
bazel run //:pmsm_sim -- my_config.json /tmp/my_run.csv
```

The simulator implementation lives in `src/`, with `main.cpp` at the project root
and tests in `test/`.

## Guides

- [Assemble a model from JSON](README.assembly.md#assemble-a-model-from-json)
- [Simulation architecture](README.architecture.md)

## Add a new C++ block type

This example adds a periodic current gain that can sit between the speed
controller and current PI. Save it as `src/blocks/current_gain.h`. The factory expects a constructor taking `const BlockSpec&` and public
members named `inputs` and `outputs`. The graph builder discovers their public
fields through C++26 reflection. Each field name is its JSON port name.
For parameters, reflect-cpp maps JSON directly to the struct and validates
fields declared with types such as `PositiveReal` and
`FiniteReal`. Reuse the block's parameter struct when it
already represents the JSON fields.

```cpp
#pragma once

#include "src/graph/block_ports.h"
#include "src/configuration.h"

namespace pmsm::drive_blocks {

struct CurrentGainInputs {
    InputPort<Current> current;
};

struct CurrentGainOutputs {
    Current current;
};

class CurrentGain {
    struct Parameters { FiniteReal gain; };

public:
    explicit CurrentGain(const BlockSpec& spec)
        : gain_(spec.read_parameters<Parameters>().gain.get()) {}

    void tick() {
        outputs.current = gain_ * inputs.current.read();
    }

    CurrentGainInputs inputs;
    CurrentGainOutputs outputs;

private:
    double gain_;
};

}  // namespace pmsm::drive_blocks
```

Add this library to [`src/blocks/BUILD.bazel`](src/blocks/BUILD.bazel).

```python
cc_library(
    name = "current_gain",
    hdrs = ["current_gain.h"],
    deps = [
        "//src/graph:block_ports",
        "//src:configuration",
    ],
)
```

Add `//src/blocks:current_gain` to the `drive` target's dependencies in
[`src/drive/BUILD.bazel`](src/drive/BUILD.bazel).

In [`src/drive/block_catalog.cpp`](src/drive/block_catalog.cpp), include `src/blocks/current_gain.h`
and append `X("current_gain", CurrentGain)` to `PMSM_BLOCK_TYPES`. Add a
trailing backslash to the previous entry. The factory checks the block
contract at compile time and registers the callback automatically. Use
`void tick()` for a periodic block or
`void advance(SimulationExecutor::Duration)` for a continuous block.
A constant source declares `static constexpr bool is_constant = true`
and can publish its outputs in its constructor. Each block chooses one
of these three forms.

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
bazel test //test/...
bazel run //:pmsm_sim -- my_config.json /tmp/current_gain.csv
```

The drive constructs blocks at stable addresses and validates the graph
before opening the CSV. During the run, callbacks read bound ports
directly. Each block keeps its state in its own instance.

## Default schedule

The example starts the PMSM from rest and advances it for 0.4 s using RK4
at a base frequency of 1 MHz, with one integration step per 1 µs tick.
The speed loop ticks every 1 ms, feedback and current PI every 50 µs,
SVPWM and inverter every 50 µs, and logging every 100 µs. The PI advances its
integrators once per tick. The inverter holds its phase voltages between
updates. The CSV records both endpoints and the latest published port values.

## Tune SimpleFOC current and velocity loops

From `pmsmcpp`, build the simulator and run the
[SimpleFOC profile](simplefoc_config.json) for a 100 rad/s velocity step.
The output includes current and velocity references, raw and filtered
feedback, dq voltage commands, and phase duties.

```sh
bazel build //:pmsm_sim
bazel run //:pmsm_sim -- simplefoc_config.json /tmp/simplefoc.csv
python3 tune_simplefoc.py current P 0.15 0.25 0.4 --target 1
python3 tune_simplefoc.py velocity P 0.04 0.08 0.16 --target 100
```

The current sweep locks the simulated shaft at rest to isolate the
electrical response. It commands a fixed q-axis current in amperes
with a zero d-axis reference and applies the candidate setting to both
current PIDs. Copy the chosen current settings into the profile before sweeping
the velocity PID. Both commands accept `P`, `I`, `D`, or `output_ramp`
and write one CSV row per candidate to standard output. A blank settling
time means the measured response did not stay within 2% of the target
through the end of the run. The tail error averages the absolute error
over the last 10% of the run. Peak current is the largest sampled
absolute value across the two dq axes. Use `--config` to sweep a copy of
the profile with different motor data, load, or duration.

The profile uses a fixed-rate model of SimpleFOC v2.3.5 with
`TorqueControlType::foc_current`, `MotionControlType::velocity`, and
centered `FOCModulationType::SpaceVectorPWM`. FOC runs at 20 kHz and
velocity at 1 kHz. When their ticks coincide, FOC uses the previous
velocity command, matching a firmware loop that calls `loopFOC()` before
`move()`. Set the rates to match your firmware. This simulation uses ideal
angle, speed, and current feedback and an averaged inverter. It models the
controller equations with fixed time steps and does not execute Arduino
firmware or simulate ADC and PWM timing.

The JSON `PID_current_d`, `PID_current_q`, and `PID_velocity` values
map to the corresponding `motor.PID_*` settings. Each `LPF_*_Tf` is in
seconds and maps to `motor.LPF_*.Tf`. The current-loop `voltage_limit`
maps to `motor.voltage_limit` in volts, and the velocity-loop
`current_limit` maps to `motor.current_limit` in amperes. The profile
assumes the driver's voltage limit equals the DC bus voltage. Treat
the example gains as simulation starting points and validate candidates
on the target hardware at conservative limits.

See the upstream
[BLDC controller](https://github.com/simplefoc/Arduino-FOC/blob/v2.3.5/src/BLDCMotor.cpp),
[PID](https://github.com/simplefoc/Arduino-FOC/blob/v2.3.5/src/common/pid.cpp),
and [filter](https://github.com/simplefoc/Arduino-FOC/blob/v2.3.5/src/common/lowpass_filter.cpp)
implementations.
