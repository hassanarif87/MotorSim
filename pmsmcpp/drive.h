#pragma once

#include "block_graph.h"
#include "blocks/continuous_pmsm.h"
#include "blocks/current_feedback.h"
#include "blocks/current_pi.h"
#include "blocks/inverter.h"
#include "blocks/speed_control.h"
#include "blocks/svpwm.h"

namespace pmsm {

namespace drive_detail {

/// Carries a scalar with a quantity tag so graph connections can check its type.
template <typename Quantity>
struct Signal {
    Signal() = default;
    Signal(double initial) : value(initial) {}

    operator double() const { return value; }

    double value = 0.0;
};

using Voltage = Signal<struct VoltageTag>;
using Current = Signal<struct CurrentTag>;
using Speed = Signal<struct SpeedTag>;
using Position = Signal<struct PositionTag>;
using Duty = Signal<struct DutyTag>;

/// Applied phase voltages at the continuous motor input, in volts.
struct PhaseVoltageInputs { InputPort<Voltage> va, vb, vc; };
/// Motor-neutral phase voltages published by the averaged inverter, in volts.
struct PhaseVoltageOutputs { Voltage va, vb, vc; };

/// Motor phase currents (A), mechanical speed (rad/s), and position (rad).
struct PlantOutputs {
    Current ia, ib, ic;
    Speed omega;
    Position theta;
};

/// Measured mechanical speed for the outer loop, in rad/s.
struct SpeedInputs { InputPort<Speed> omega; };
/// Requested q-axis current from the speed controller, in amperes.
struct SpeedOutputs { Current iq_ref; };

/// Phase currents (A) and mechanical position (rad) sampled for the dq transform.
struct FeedbackInputs {
    InputPort<Current> ia, ib, ic;
    InputPort<Position> theta;
};

/// Sampled d- and q-axis currents, in amperes.
struct FeedbackOutputs { Current id, iq; };

/// q-axis reference and measured dq currents (A), plus mechanical speed (rad/s).
struct PiInputs {
    InputPort<Current> iq_ref, id, iq;
    InputPort<Speed> omega;
};

/// Voltage-limited dq commands held between current-loop ticks, in volts.
struct PiOutputs { Voltage vd, vq; };

/// Latest dq voltage command (V) and mechanical position (rad) sampled by SVPWM.
struct PwmInputs {
    InputPort<Voltage> vd, vq;
    InputPort<Position> theta;
};

/// Normalized phase duty commands consumed by the inverter.
struct DutyInputs { InputPort<Duty> duty_a, duty_b, duty_c; };
/// Normalized phase duty commands published by SVPWM.
struct DutyOutputs { Duty duty_a, duty_b, duty_c; };

// Reflected member names form the port portion of each JSON endpoint.
BOOST_DESCRIBE_STRUCT(PhaseVoltageInputs, (), (va, vb, vc))
BOOST_DESCRIBE_STRUCT(PhaseVoltageOutputs, (), (va, vb, vc))
BOOST_DESCRIBE_STRUCT(PlantOutputs, (), (ia, ib, ic, omega, theta))
BOOST_DESCRIBE_STRUCT(SpeedInputs, (), (omega))
BOOST_DESCRIBE_STRUCT(SpeedOutputs, (), (iq_ref))
BOOST_DESCRIBE_STRUCT(FeedbackInputs, (), (ia, ib, ic, theta))
BOOST_DESCRIBE_STRUCT(FeedbackOutputs, (), (id, iq))
BOOST_DESCRIBE_STRUCT(PiInputs, (), (iq_ref, id, iq, omega))
BOOST_DESCRIBE_STRUCT(PiOutputs, (), (vd, vq))
BOOST_DESCRIBE_STRUCT(PwmInputs, (), (vd, vq, theta))
BOOST_DESCRIBE_STRUCT(DutyInputs, (), (duty_a, duty_b, duty_c))
BOOST_DESCRIBE_STRUCT(DutyOutputs, (), (duty_a, duty_b, duty_c))

/// Stores bound input ports and the last outputs published by a block.
template <typename Inputs, typename Outputs>
struct Ports {
    Inputs inputs;
    Outputs outputs;
};

}  // namespace drive_detail

/**
 * Connects a continuous PMSM to sampled speed, current, and inverter blocks.
 *
 * The executor supplies the clock and callback order. Each callback reads the
 * latest values published to its bound ports. The motor must outlive the drive,
 * and the drive must stay at a stable address while callbacks are registered.
 */
class CurrentControlledPmsm {
public:
    /**
     * Binds JSON [destination, source] connections and publishes the initial motor state.
     * The graph registry exists only during construction.
     *
     * @param bus_voltage DC link voltage in volts.
     * @param speed_reference Mechanical speed command in rad/s.
     * @param id_reference Constant d-axis current command in amperes.
     * @param current_period Sample interval used by the digital PI integrators.
     * @throws std::invalid_argument If a connection is invalid or a required input is unbound.
     */
    CurrentControlledPmsm(ContinuousPmsm& motor, CurrentPi controller, SpeedControl speed,
                          double bus_voltage, double speed_reference, double id_reference,
                          SimulationExecutor::Duration current_period, const Connections& connections)
        : motor_(motor), controller_(controller, std::chrono::duration<double>(current_period).count()),
          speed_(speed), bus_voltage_(bus_voltage), speed_reference_(speed_reference),
          id_reference_(id_reference) {
        BlockGraph graph;
        graph.add("pmsm", plant_.inputs, plant_.outputs);
        graph.add("speed_control", speed_ports_.inputs, speed_ports_.outputs);
        graph.add("current_feedback", feedback_.inputs, feedback_.outputs);
        graph.add("current_pi", pi_.inputs, pi_.outputs);
        graph.add("svpwm", pwm_.inputs, pwm_.outputs);
        graph.add("inverter", inverter_.inputs, inverter_.outputs);
        graph.connect(connections);
        publish_motor_outputs();
    }

    CurrentControlledPmsm(const CurrentControlledPmsm&) = delete;
    CurrentControlledPmsm& operator=(const CurrentControlledPmsm&) = delete;

    /// Holds the sampled phase voltages through one RK4 step and publishes the accepted outputs.
    void advance(SimulationExecutor::Duration interval) {
        const auto& in = plant_.inputs;
        motor_.set_voltage({{in.va.read(), in.vb.read(), in.vc.read()}});
        motor_.advance(interval);
        publish_motor_outputs();
    }

    /// Samples speed and updates the held q-axis current reference.
    void tick_speed() {
        speed_ports_.outputs.iq_ref = speed_.output(speed_reference_, speed_ports_.inputs.omega.read());
    }

    /// Samples phase currents and transforms them using the electrical rotor angle.
    void tick_feedback() {
        const auto& in = feedback_.inputs;
        const auto measured = current_feedback({{in.ia.read(), in.ib.read(), in.ic.read()}},
                                                motor_.pole_pairs() * in.theta.read());
        feedback_.outputs = {measured.d, measured.q};
    }

    /// Publishes the PI voltage command and advances its integrators by one sample interval.
    void tick_current() {
        const auto& in = pi_.inputs;
        const auto command = controller_.tick({
            {id_reference_, in.iq_ref.read()}, {in.id.read(), in.iq.read()},
            in.omega.read(), bus_voltage_,
        });
        pi_.outputs = {command.d, command.q};
    }

    /// Converts the held voltage command to duty cycles using rotor position at this tick.
    void tick_svpwm() {
        const auto& in = pwm_.inputs;
        const auto duty = svpwm({in.vd.read(), in.vq.read()}, motor_.pole_pairs() * in.theta.read(),
                                bus_voltage_);
        pwm_.outputs = {duty.abc[0], duty.abc[1], duty.abc[2]};
    }

    /// Converts duty cycles into averaged phase voltages and holds them until the next tick.
    void tick_inverter() {
        const auto& in = inverter_.inputs;
        const auto voltage = ideal_inverter({{in.duty_a.read(), in.duty_b.read(), in.duty_c.read()}},
                                            bus_voltage_);
        inverter_.outputs = {voltage.abc[0], voltage.abc[1], voltage.abc[2]};
    }

    /// Returns outputs from the latest accepted motor state.
    const auto& motor_outputs() const { return plant_.outputs; }
    /// Returns the most recent dq current sample.
    const auto& feedback_outputs() const { return feedback_.outputs; }
    /// Returns the voltage commands from the most recent PI tick.
    const auto& pi_outputs() const { return pi_.outputs; }
    /// Returns the most recent SVPWM duty cycles.
    const auto& pwm_outputs() const { return pwm_.outputs; }
    /// Returns the most recent averaged inverter voltages.
    const auto& inverter_outputs() const { return inverter_.outputs; }
    /// Returns the fixed d-axis reference and latest connected q-axis reference.
    DqCurrent current_reference() const { return {id_reference_, pi_.inputs.iq_ref.read()}; }

private:
    void publish_motor_outputs() {
        const auto& out = motor_.output();
        plant_.outputs = {out.current.abc[0], out.current.abc[1], out.current.abc[2],
                          out.omega, out.theta};
    }

    ContinuousPmsm& motor_;
    DigitalCurrentPi controller_;
    SpeedControl speed_;
    double bus_voltage_;
    double speed_reference_;
    double id_reference_;
    drive_detail::Ports<drive_detail::PhaseVoltageInputs, drive_detail::PlantOutputs> plant_;
    drive_detail::Ports<drive_detail::SpeedInputs, drive_detail::SpeedOutputs> speed_ports_;
    drive_detail::Ports<drive_detail::FeedbackInputs, drive_detail::FeedbackOutputs> feedback_;
    drive_detail::Ports<drive_detail::PiInputs, drive_detail::PiOutputs> pi_;
    drive_detail::Ports<drive_detail::PwmInputs, drive_detail::DutyOutputs> pwm_;
    drive_detail::Ports<drive_detail::DutyInputs, drive_detail::PhaseVoltageOutputs> inverter_;
};

}  // namespace pmsm
