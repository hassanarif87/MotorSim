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

The defaults reproduce the Python speed-control example. The motor starts at
rest with zero external load torque. RK4 advances the motor and current PI
integrators every 10 µs for 0.4 s. The CSV records samples every 100 µs,
including both endpoints. The CSV includes
mechanical speed and position, phase and dq currents, current references, voltage
commands, phase voltages, and inverter duty cycles.

Each model in `blocks/` has its own Bazel library target. For example, a plant-only
simulation can depend on `//blocks:plant` and supply phase voltages and load torque
directly. `drive.h` wires the independent blocks into the current loop. Motor and
controller state are supplied by the caller at each evaluation. Eigen provides
fixed-size phase vectors, PI vector math, and integration state.
