#pragma once

#include "block_graph.h"
#include "configuration.h"
#include "blocks/continuous_pmsm.h"
#include "blocks/current_feedback.h"
#include "blocks/current_pi.h"
#include "blocks/inverter.h"
#include "blocks/speed_control.h"
#include "blocks/svpwm.h"

namespace pmsm {

namespace drive_blocks {

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
struct PiOutputs { Voltage vd, vq; Current id_ref; };

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
struct NoInputs {};
struct ConstantCurrentOutputs { Current value; };

BOOST_DESCRIBE_STRUCT(NoInputs, (), ())
BOOST_DESCRIBE_STRUCT(ConstantCurrentOutputs, (), (value))
BOOST_DESCRIBE_STRUCT(PhaseVoltageInputs, (), (va, vb, vc))
BOOST_DESCRIBE_STRUCT(PhaseVoltageOutputs, (), (va, vb, vc))
BOOST_DESCRIBE_STRUCT(PlantOutputs, (), (ia, ib, ic, omega, theta))
BOOST_DESCRIBE_STRUCT(SpeedInputs, (), (omega))
BOOST_DESCRIBE_STRUCT(SpeedOutputs, (), (iq_ref))
BOOST_DESCRIBE_STRUCT(FeedbackInputs, (), (ia, ib, ic, theta))
BOOST_DESCRIBE_STRUCT(FeedbackOutputs, (), (id, iq))
BOOST_DESCRIBE_STRUCT(PiInputs, (), (iq_ref, id, iq, omega))
BOOST_DESCRIBE_STRUCT(PiOutputs, (), (vd, vq, id_ref))
BOOST_DESCRIBE_STRUCT(PwmInputs, (), (vd, vq, theta))
BOOST_DESCRIBE_STRUCT(DutyInputs, (), (duty_a, duty_b, duty_c))
BOOST_DESCRIBE_STRUCT(DutyOutputs, (), (duty_a, duty_b, duty_c))

/// Stores bound input ports and the last outputs published by a block.
using config_detail::bounded_number;
using config_detail::number;
using config_detail::positive_integer;

class Motor {
public:
    explicit Motor(const BlockSpec& spec)
        : motor_(PmsmPlant(spec.parameters.get<MotorParameters>()),
                 spec.parameters.contains("load_torque_nm")
                     ? number(spec.parameters, "load_torque_nm") : 0.0) {
        publish();
    }

    void advance(SimulationExecutor::Duration interval) {
        motor_.set_voltage({{inputs.va.read(), inputs.vb.read(), inputs.vc.read()}});
        motor_.advance(interval);
        publish();
    }

    PhaseVoltageInputs inputs;
    PlantOutputs outputs;

private:
    void publish() {
        const auto& out = motor_.output();
        outputs = {out.current.abc[0], out.current.abc[1], out.current.abc[2], out.omega, out.theta};
    }

    ContinuousPmsm motor_;
};

class SpeedController {
public:
    explicit SpeedController(const BlockSpec& spec)
        : controller_(bounded_number(spec.parameters, "gain_a_per_rad_s", true),
                      bounded_number(spec.parameters, "max_current_a", true)),
          reference_(spec.parameters.at("reference_rad_s").is_null()
              ? bounded_number(spec.parameters, "bus_voltage_v", false) /
                (std::sqrt(3.0) * bounded_number(spec.parameters, "back_emf_v_per_rad_s", false))
              : number(spec.parameters, "reference_rad_s")) {}

    void tick() { outputs.iq_ref = controller_.output(reference_, inputs.omega.read()); }

    SpeedInputs inputs;
    SpeedOutputs outputs;

private:
    SpeedControl controller_;
    double reference_;
};

class Feedback {
public:
    explicit Feedback(const BlockSpec& spec)
        : pole_pairs_(positive_integer(spec.parameters, "pole_pairs")) {}

    void tick() {
        const auto measured = current_feedback(
            {{inputs.ia.read(), inputs.ib.read(), inputs.ic.read()}},
            pole_pairs_ * inputs.theta.read());
        outputs = {measured.d, measured.q};
    }

    FeedbackInputs inputs;
    FeedbackOutputs outputs;

private:
    int pole_pairs_;
};

class CurrentController {
public:
    explicit CurrentController(const BlockSpec& spec)
        : controller_(CurrentPi({
              bounded_number(spec.parameters, "kp", true), bounded_number(spec.parameters, "ki", true),
              bounded_number(spec.parameters, "kaw", true),
              bounded_number(spec.parameters, "inductance_h", false),
              positive_integer(spec.parameters, "pole_pairs"),
              bounded_number(spec.parameters, "back_emf_v_per_rad_s", false),
          }), std::chrono::duration<double>(*spec.period).count()),
          bus_voltage_(bounded_number(spec.parameters, "bus_voltage_v", false)) {
        outputs.id_ref = number(spec.parameters, "id_reference_a");
    }

    void tick() {
        const auto command = controller_.tick({
            {outputs.id_ref, inputs.iq_ref.read()}, {inputs.id.read(), inputs.iq.read()},
            inputs.omega.read(), bus_voltage_,
        });
        outputs.vd = command.d;
        outputs.vq = command.q;
    }

    PiInputs inputs;
    PiOutputs outputs;

private:
    DigitalCurrentPi controller_;
    double bus_voltage_;
};

class Pwm {
public:
    explicit Pwm(const BlockSpec& spec)
        : pole_pairs_(positive_integer(spec.parameters, "pole_pairs")),
          bus_voltage_(bounded_number(spec.parameters, "bus_voltage_v", false)) {}

    void tick() {
        const auto duty = svpwm({inputs.vd.read(), inputs.vq.read()},
                                pole_pairs_ * inputs.theta.read(), bus_voltage_);
        outputs = {duty.abc[0], duty.abc[1], duty.abc[2]};
    }

    PwmInputs inputs;
    DutyOutputs outputs;

private:
    int pole_pairs_;
    double bus_voltage_;
};

class Inverter {
public:
    explicit Inverter(const BlockSpec& spec)
        : bus_voltage_(bounded_number(spec.parameters, "bus_voltage_v", false)) {}

    void tick() {
        const auto voltage = ideal_inverter(
            {{inputs.duty_a.read(), inputs.duty_b.read(), inputs.duty_c.read()}}, bus_voltage_);
        outputs = {voltage.abc[0], voltage.abc[1], voltage.abc[2]};
    }

    DutyInputs inputs;
    PhaseVoltageOutputs outputs;

private:
    double bus_voltage_;
};

struct ConstantCurrent {
    explicit ConstantCurrent(const BlockSpec& spec)
        : outputs{number(spec.parameters, "value_a")} {}

    NoInputs inputs;
    ConstantCurrentOutputs outputs;
};

}  // namespace drive_blocks
}  // namespace pmsm
