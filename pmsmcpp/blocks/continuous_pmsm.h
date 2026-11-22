#pragma once

#include <chrono>

#include "blocks/plant.h"
#include "rk4.h"
#include "simulation_executor.h"

#include "block_ports.h"
#include "configuration.h"

namespace pmsm {

class ContinuousPmsm {
public:
    explicit ContinuousPmsm(PmsmPlant plant, double load_torque, MotorState initial = {})
        : plant_(plant), state_(initial), input_{{Eigen::Vector3d::Zero()}, load_torque},
          output_(plant_.output(state_)) {}

    void set_voltage(const PhaseVoltage& voltage) { input_.voltage = voltage; }
    int pole_pairs() const { return plant_.pole_pairs(); }
    const MotorOutput& output() const { return output_; }

    void advance(SimulationExecutor::Duration interval) {
        const State initial(state_.iq, state_.id, state_.omega, state_.theta);
        const auto derivative = [this](const State& trial) {
            const auto rate = plant_.derivative({trial[0], trial[1], trial[2], trial[3]}, input_);
            return State(rate.iq, rate.id, rate.omega, rate.theta);
        };
        const auto accepted = rk4_step(initial, std::chrono::duration<double>(interval).count(),
                                       derivative);
        state_ = {accepted[0], accepted[1], accepted[2], accepted[3]};
        post_integrate();
    }

private:
    using State = Eigen::Vector4d;

    void post_integrate() { output_ = plant_.output(state_); }

    PmsmPlant plant_;
    MotorState state_;
    MotorInput input_;
    MotorOutput output_;
};

namespace drive_blocks {

/// Applied phase voltages at the continuous motor input, in volts.
struct PhaseVoltageInputs { InputPort<Voltage> va, vb, vc; };
BOOST_DESCRIBE_STRUCT(PhaseVoltageInputs, (), (va, vb, vc))

/// Motor phase currents (A), mechanical speed (rad/s), and position (rad).
struct PlantOutputs {
    Current ia, ib, ic;
    Speed omega;
    Position theta;
};
BOOST_DESCRIBE_STRUCT(PlantOutputs, (), (ia, ib, ic, omega, theta))

using config_detail::bounded_number;
using config_detail::number;
using config_detail::positive_integer;

class Motor {
public:
    explicit Motor(const BlockSpec& spec)
        : motor_(PmsmPlant(load_parameters(spec.parameters)),
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
    static MotorParameters load_parameters(const nlohmann::json& parameters) {
        return {
            bounded_number(parameters, "resistance_ohm", true),
            bounded_number(parameters, "inductance_h", false),
            positive_integer(parameters, "pole_pairs"),
            bounded_number(parameters, "back_emf_v_per_rad_s", false),
            bounded_number(parameters, "torque_constant_nm_per_a", true),
            bounded_number(parameters, "inertia_kg_m2", false),
            bounded_number(parameters, "viscous_friction_nm_per_rad_s", true),
        };
    }

    void publish() {
        const auto& out = motor_.output();
        outputs = {out.current.abc[0], out.current.abc[1], out.current.abc[2], out.omega, out.theta};
    }

    ContinuousPmsm motor_;
};

}  // namespace drive_blocks

}  // namespace pmsm
