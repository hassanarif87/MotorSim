#pragma once

#include <chrono>

#include "graph/block_ports.h"
#include "blocks/transforms.h"
#include "configuration.h"
#include "rk4.h"
#include "simulation_executor.h"

namespace pmsm {

struct MotorParameters {
    NonnegativeReal resistance_ohm{0.03};
    PositiveReal inductance_h{0.03};
    PositiveInt pole_pairs{1};
    PositiveReal back_emf_v_per_rad_s{0.1};
    NonnegativeReal torque_constant_nm_per_a{0.1};
    PositiveReal inertia_kg_m2{0.03};
    NonnegativeReal viscous_friction_nm_per_rad_s{0.0};
    std::optional<FiniteReal> load_torque_nm;
};

struct MotorState {
    double iq;
    double id;
    double omega;
    double theta;
};

struct MotorInput {
    PhaseVoltage voltage;
    double load_torque;
};

struct MotorOutput {
    PhaseCurrent current;
    double omega;
    double theta;
};

class PmsmPlant {
public:
    explicit PmsmPlant(MotorParameters parameters) : parameters_(parameters) {}

    int pole_pairs() const { return parameters_.pole_pairs.get(); }

    MotorOutput output(const MotorState& state) const {
        const double theta_e = parameters_.pole_pairs.get() * state.theta;
        return {{inverse_clarke_park(theta_e, state.id, state.iq)},
                state.omega, state.theta};
    }

    MotorState derivative(const MotorState& state, const MotorInput& input) const {
        const auto& p = parameters_;
        const double omega_e = p.pole_pairs.get() * state.omega;
        const auto voltage = clarke_park(p.pole_pairs.get() * state.theta, input.voltage.abc);
        const double iq_dot =
            (voltage.y() - p.resistance_ohm.get() * state.iq - omega_e * p.inductance_h.get() * state.id
             - p.back_emf_v_per_rad_s.get() * state.omega) / p.inductance_h.get();
        const double id_dot =
            (voltage.x() - p.resistance_ohm.get() * state.id + omega_e * p.inductance_h.get() * state.iq)
            / p.inductance_h.get();
        const double torque = 1.5 * p.torque_constant_nm_per_a.get() * state.iq;
        const double omega_dot =
            (torque - input.load_torque - p.viscous_friction_nm_per_rad_s.get() * state.omega) / p.inertia_kg_m2.get();
        return {iq_dot, id_dot, omega_dot, state.omega};
    }

private:
    MotorParameters parameters_;
};

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

/// Motor phase currents (A), mechanical speed (rad/s), and position (rad).
struct PlantOutputs {
    Current ia, ib, ic;
    Speed omega;
    Position theta;
};

class Motor {
public:
    explicit Motor(const BlockSpec& spec) : Motor(spec.read_parameters<MotorParameters>()) {}

    void advance(SimulationExecutor::Duration interval) {
        motor_.set_voltage({{inputs.va.read(), inputs.vb.read(), inputs.vc.read()}});
        motor_.advance(interval);
        publish();
    }

    PhaseVoltageInputs inputs;
    PlantOutputs outputs;

private:
    explicit Motor(const MotorParameters& parameters)
        : motor_(PmsmPlant(parameters),
                 parameters.load_torque_nm ? parameters.load_torque_nm->get() : 0.0) {
        publish();
    }

    void publish() {
        const auto& out = motor_.output();
        outputs = {out.current.abc[0], out.current.abc[1], out.current.abc[2], out.omega, out.theta};
    }

    ContinuousPmsm motor_;
};

}  // namespace drive_blocks

}  // namespace pmsm
