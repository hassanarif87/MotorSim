# PMSM C++ simulation

The simulator builds with GCC 16.2 using `-std=c++26 -freflection`. Bazel selects `gcc-16`
from `PATH`. From the repository root, run the following with Bazelisk or
Bazel 8.4.0.

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
controller and current PI. Save it as `blocks/current_gain.h`. The factory expects a constructor taking `const BlockSpec&` and public
members named `inputs` and `outputs`. The graph builder discovers their public
fields through C++26 reflection. Each field name is its JSON port name.

```cpp
#pragma once

#include "block_ports.h"
#include "configuration.h"

namespace pmsm::drive_blocks {

struct CurrentGainInputs {
    InputPort<Current> current;
};

struct CurrentGainOutputs {
    Current current;
};

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

}  // namespace pmsm::drive_blocks
```

Add this library to [`blocks/BUILD.bazel`](blocks/BUILD.bazel).

```python
cc_library(
    name = "current_gain",
    hdrs = ["current_gain.h"],
    deps = [
        "//:block_ports",
        "//:configuration",
    ],
)
```

Add `//blocks:current_gain` to the `drive` target's dependencies in
[`BUILD.bazel`](BUILD.bazel).

In [`block_catalog.cpp`](block_catalog.cpp), include `blocks/current_gain.h`
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
