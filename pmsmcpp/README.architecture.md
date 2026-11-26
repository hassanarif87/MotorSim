# PMSM simulator architecture

The simulator combines a continuous motor plant with digital blocks that run
at configured rates. A JSON file defines the block instances, signal
connections, and CSV columns. See the [assembly guide](README.assembly.md)
for instructions on editing the model. Run the Bazel tests in [`test/`](test/)
with `bazel test //test/...`.

## Startup and ownership

| File | Responsibility |
| --- | --- |
| [`main.cpp`](main.cpp) | Loads the config, constructs the drive, and registers the CSV logger. |
| [`configuration.h`](src/configuration.h) | Uses reflect-cpp to load JSON into typed simulation and block specifications. Converts digital rates in Hz to integer nanosecond periods. |
| [`drive/drive.h`](src/drive/drive.h) | Constructs the configured model. Owns block instances and the executor and binds callbacks at startup. |
| [`drive/block_catalog.cpp`](src/drive/block_catalog.cpp) | Lists the block types available to JSON configs and maps each type name to its factory. |
| [`graph/block_ports.h`](src/graph/block_ports.h) | Defines the shared typed input port and signal quantities used by components. |
| [`graph/block_graph.h`](src/graph/block_graph.h) | Uses C++26 reflection to discover named ports, validate connections, and resolve CSV readers. |
| [`src/blocks/`](src/blocks/) | Defines each component's equations, connectable wrapper, and reflected ports in its own header. |
| [`simulation_executor.h`](src/simulation_executor.h) | Advances simulation time and dispatches the continuous and digital callbacks. |

Each factory reads its block's parameter object into a local C++ struct.
The graph's name maps exist during drive construction. Once validation
succeeds, each input holds a typed pointer to its source value. CSV readers
also retain the addresses they resolve at startup. Blocks publish values into
their own output storage, so a consumer sees the latest published value when
it reads its input. Each block owns its internal state.

The factory uses executor concepts to identify a block's `tick()` or
`advance(dt)` method and register it automatically. The executor stores
periodic callbacks in a fixed-capacity array. A delegate holds an
object pointer and a dispatch function pointer. Binding and calling these
delegates requires no heap allocation. The drive owns the block instances
for the duration of the run, and the logger stays alive through the run.

## Signal flow

The default config closes a speed loop around a current loop. The motor
also supplies rotor position to current feedback and SVPWM, and mechanical
speed to the PI for feedforward.

```mermaid
flowchart LR
    Motor[PMSM] -->|speed| Speed[Speed control]
    Motor -->|phase currents and position| Feedback[Current feedback]
    Speed -->|q-axis reference| PI[Current PI]
    Feedback -->|dq currents| PI
    PI -->|dq voltage| PWM[SVPWM]
    PWM -->|duty cycles| Inverter[Inverter]
    Inverter -->|phase voltages| Motor
```

## Simulation time and integration

1. The motor publishes its initial outputs during construction. All periodic
   callbacks are first due at simulated time zero.
2. At each integer tick, the executor runs a callback when the tick index
   modulo its configured stride is zero. It runs due callbacks in ascending priority,
   using registration order for ties. With the default config, coincident
   ticks run speed control, feedback, current PI, SVPWM, inverter, then CSV
   logging.
3. The executor advances the motor by one fixed interval between ticks.
   If the requested duration falls between ticks, it advances the motor
   through the remaining fraction of the interval after the last tick.
4. The motor reads its phase-voltage inputs once per interval.
   [`ContinuousPmsm`](src/blocks/continuous_pmsm.h) runs the four RK4 stages with
   those held voltages and commits the accepted state. It computes phase
   currents, speed, and position before the next digital callbacks run.

[`PmsmPlant`](src/blocks/continuous_pmsm.h) provides the motor equations. The digital
current PI advances its own integrators once per current tick. SVPWM and the
averaged inverter publish duty cycles and voltages that remain available
until their next ticks.

## Results

The [CSV logger](src/csv_logger.h) reads the configured ports on its own clock. It records
`time_s` from the executor, then the latest values of the selected inputs
and outputs. It samples at zero and at the final time, including a final row
when the end falls between logging ticks. The
[XY notebook](plot_pmsm_results.ipynb) reads the generated CSV for analysis.
