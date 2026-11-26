# Dead-time compensation and load-torque feedforward

## PWM dead-time compensation

[DeadTimeCompensation](src/blocks/dead_time_compensation.h) runs after SVPWM and before the inverter captures the duty commands. For each switching leg, it uses the measured phase current to estimate the average duty lost or gained during dead time. Positive current flows into the motor. The block increases duty for positive current and decreases it for negative current.

```text
polarity = clamp(i_phase / zero_current_band_a, -1, 1)
duty_corrected = clamp(duty_svpwm + dead_time_s * pwm_rate_hz * polarity, 0, 1)
```

The **zero-current band** is a linear taper around zero. At zero current the correction is zero. At half the band threshold it is half strength. At or above the threshold in either direction it is full strength. A leg commanded fully on or fully off remains at that rail because it has no switching edges.

The [compensated profile](configs/simplefoc_dead_time_compensation_config.json) uses 20 kHz PWM, 0.5 µs dead time, and a 0.05 A threshold. The full correction is **±1 percentage point of duty**. At ±25 mA it is **±0.5 percentage points**. The CSV logs raw commands as `duty_a`, `duty_b`, and `duty_c`, and logs corrected commands with the `_compensated` suffix.

The inverter applies the corrected duties at the next PWM boundary, 50 µs later. Current polarity can change before that boundary, leaving residual error. The correction also loses authority near the duty limits. Use the [uncompensated profile](configs/simplefoc_nonideal_inverter_config.json) as the baseline. Both profiles include inverter dead time, switch resistance, and the latch delay.

## Load-torque feedforward for velocity and position

The following design is proposed. The current [SimpleFocVelocity block](src/blocks/simplefoc.h) generates its q-axis current reference from velocity feedback.

Use the load model and motion reference to estimate the required shaft torque. Convert that estimate to q-axis current and add it to the velocity PI contribution before the final current clamp.

```text
torque_ff = J * alpha_ref + B * omega_ref + torque_load(theta_ref, omega_ref)
iq_ff     = torque_ff / Kt_effective
iq_ref    = clamp(iq_pi + iq_ff, -current_limit, current_limit)
```

Use mechanical units throughout. In the [MotorSim plant](src/blocks/continuous_pmsm.h), positive load torque is subtracted from motor torque, and `Kt_effective = 1.5 * torque_constant_nm_per_a`. With the example profile's torque constant, a +0.01 N·m opposing load calls for about +0.376 A of feedforward current.

- **Velocity control** uses the known opposing torque and modeled viscous friction. Include the inertia term when the speed reference comes from a trajectory with known acceleration.
- **Position control** uses the position loop to generate a velocity correction. Evaluate position-dependent load torque, such as gravity, at the reference angle. At a stationary target, the gravity term can supply holding current while the reference velocity and acceleration are zero.

Feed the combined command's saturation back into the velocity PI anti-windup logic. The feedforward uses part of the available current, so the feedback loop can only use the remaining headroom in the direction of saturation. This torque feedforward acts at the current reference. The existing back-EMF and cross-coupling feedforward acts at the inner loop's voltage command.
