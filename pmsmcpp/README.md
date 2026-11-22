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

Edit the JSON to change motor constants, current PI gains, the DC bus voltage,
the speed controller, load torque, and logging. Set `speed_control.reference_rad_s`
to a number for a fixed speed command, or use `null` to compute the ideal no-load
speed from the configured bus voltage and back-EMF constant.

The defaults use the Python speed-control example's motor parameters and initial
conditions. Only the PMSM state is integrated. The executor runs the speed loop
every 1 ms, current PI and PWM/inverter every 50 µs, and CSV logging every 100 µs.
Set each clock's rate in Hz using `speed_control.rate_hz`, `current_pi.rate_hz`,
`inverter.pwm_rate_hz`, and `simulation.log_rate_hz`. Rates must be positive
integers that divide 1,000,000,000 exactly so every tick lands on a whole
nanosecond. Older configurations can still specify periods in nanoseconds or
use `log_every_steps` for logging. A rate in Hz takes precedence over its
legacy period.

RK4 advances the plant for 0.4 s. The maximum step is `simulation.step_s`
(10 µs by default), and the executor shortens steps to land on clock events.
At a shared timestamp it publishes plant outputs, then runs speed control,
current control, PWM, and logging in that order. The current PI advances its
integrators once per current tick. Phase voltages are held between PWM ticks.

The CSV includes both endpoints. Phase currents, speed, and position come from
the accepted plant state. The dq current columns report the last current sample,
and the command, duty, and voltage columns report held values.

Each model in `blocks/` has its own Bazel library target. For example, a plant-only
simulation can depend on `//blocks:plant` and supply phase voltages and load torque
directly. Use `//blocks:continuous_pmsm` for a standalone motor block that owns
its state, runs RK4, and publishes accepted outputs. `drive.h` wires the
independent blocks into the current loop. The executor
stores member callbacks in a fixed-capacity registry and dispatches synchronously.
Bound objects must stay alive at a stable address through the simulation run.
