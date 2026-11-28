# PMSM C++ simulation

The simulator builds with GCC 16.2 in C++26 mode. The graph targets use
`-freflection`, and Bazel selects `gcc-16`
from `PATH`. From the repository root, run the following with Bazelisk or
Bazel 8.4.0.

```sh
cd pmsmcpp
bazel build //:pmsm_sim
bazel test //test/...
bazel run //:pmsm_sim -- configs/simplefoc_config.json
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
#include "src/exec/configuration.h"

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

The [`//src/blocks` library](src/blocks/BUILD.bazel) picks up the new header automatically.

In [`src/drive/block_catalog.cpp`](src/drive/block_catalog.cpp), include `src/blocks/current_gain.h`
and append `X(CurrentGain)` to `PMSM_BLOCK_TYPES`. Add a
trailing backslash to the previous entry. The factory checks the block
contract at compile time and registers the block automatically. A periodic block
implements `void tick()`. A continuous block declares `state_size` and implements
`initial_state()`, `publish(state)`, and a const `derivative(state, rate)`.
A constant source declares `static constexpr bool is_constant = true`
and can publish its outputs in its constructor. Each block chooses one
of these three forms.

Start with a fresh copy of `configs/simplefoc_config.json` and add this instance to
`blocks`.

```json
{
  "name": "reference_gain",
  "type": "CurrentGain",
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

The gain runs before the PI when their ticks coincide. Once the speed
controller publishes its 3 A startup request, the gain passes 1.5 A to
the current controller. Rebuild and check the run with the commands below.

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
[SimpleFOC profile](configs/simplefoc_config.json) for a 100 rad/s velocity step.
The output includes current and velocity references, raw and filtered
feedback, dq voltage commands, and phase duties.

```sh
bazel build //:pmsm_sim
bazel run //:pmsm_sim -- configs/simplefoc_config.json /tmp/simplefoc.csv
python3 scripts/tune_simplefoc.py current P 0.15 0.25 0.4 --target 1
python3 scripts/tune_simplefoc.py velocity P 0.04 0.08 0.16 --target 100
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

The current controller uses measured mechanical speed and dq currents to
compensate for back-EMF and cross-axis coupling. Match its `pole_pairs`,
`inductance_h`, and `back_emf_v_per_rad_s` to the plant. The voltage limit
and output ramp apply to the combined PID and feedforward command.
Set `feedforward_enabled` to `false` in the current controller's parameters
to disable both compensation terms. It defaults to `true`.

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

## Run the SimpleFOC speed loop with a nonideal inverter

```sh
bazel run //:pmsm_sim -- configs/simplefoc_nonideal_inverter_config.json /tmp/simplefoc_nonideal.csv
```

`NonIdealInverter` accepts the same three duty inputs and produces the same three
motor-neutral voltage outputs as `Inverter`. Set its `rate_hz` to the PWM carrier
rate and schedule it after the PWM command block when their ticks coincide.
The block captures the new duty command at each tick and applies it at the
following PWM boundary. At 20 kHz this adds 50 µs of delay. The initial duty is
50% on all three legs.

The profile uses example values of 0.5 µs for `dead_time_s` and 20 mΩ for
`switch_resistance_ohm`. Connect `ia`, `ib`, and `ic` from the plant and specify
both parameters. Set both to zero to model the PWM latch delay alone. The
bridge uses cycle-averaged voltages and a fixed DC bus.

Use the profile with `scripts/tune_simplefoc.py --config configs/simplefoc_nonideal_inverter_config.json`.

## Run the SimpleFOC speed loop with dead-time compensation

```sh
bazel run //:pmsm_sim -- configs/simplefoc_dead_time_compensation_config.json /tmp/simplefoc_compensated.csv
```

This profile runs `DeadTimeCompensation` between SVPWM and the inverter at the
PWM rate. Set the block's `dead_time_s` to match the inverter's dead time.
With the example settings, it adds or subtracts up to one percentage point of
duty based on phase current. The correction tapers linearly to zero within
`zero_current_band_a` (0.05 A). Set the compensation block's `dead_time_s` to
zero for a baseline run with the inverter's dead time still active. The CSV
includes the raw duties and the `duty_*_compensated` commands before the latch.

## Run the SimpleFOC speed loop with a switching inverter

```sh
bazel run //:pmsm_sim -- configs/simplefoc_switching_inverter_config.json /tmp/simplefoc_switching.csv
```

`PwmSignals` converts SVPWM duties to six complementary, center-aligned
gate commands. A duty captured at one carrier boundary is applied at the
next boundary. When a leg changes state, the outgoing switch turns off
immediately and the incoming switch waits for the configured dead time.

`SwitchingInverter` computes the three motor-neutral phase voltages from the
gates and phase currents. It models on-state resistance and a fixed diode
drop during dead time. When both gates are off and phase current is exactly
zero, the model uses the bus midpoint for that pole.

The profile uses a 20 kHz carrier and a 100 ns simulation tick, so the
500 ns dead time spans five ticks. PWM pulse widths are rounded to the
nearest simulation tick. Set `log_rate_hz` to `10000000` for a short run if
you need to inspect every gate transition.

## Compare switching-inverter dead-time compensation

```sh
bazel run //:pmsm_sim -- configs/simplefoc_switching_inverter_config.json /tmp/switching_off.csv
bazel run //:pmsm_sim -- configs/simplefoc_switching_dead_time_compensation_config.json /tmp/switching_on.csv
```

Both profiles keep 500 ns of gate dead time at a 20 kHz PWM carrier. The
compensated profile adjusts the duty commands between SVPWM and `PwmSignals`
and logs the adjusted values as `duty_*_compensated`.

Over the final 200 ms of the 400 ms run, compensation reduced speed RMS
tracking error from 3.01 to 2.17 rad/s and q-axis current error from 0.171 to
0.134 A. The d-axis current error increased from 0.0151 to 0.0768 A. Current
errors use the measured dq currents and their logged references.

[Switching-inverter comparison plot](simplefoc_switching_dead_time_compensation_comparison.png)
