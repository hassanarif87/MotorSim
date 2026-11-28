#pragma once

#include <optional>

#include "src/blocks/transforms.h"
#include "src/exec/configuration.h"
#include "src/exec/block_interface.h"
#include "src/graph/block_ports.h"

namespace pmsm {

struct MotorParameters {
    NonnegativeReal resistance_ohm{0.03};
    PositiveReal inductance_h{0.03};
    PositiveInt pole_pairs{1};
    PositiveReal back_emf_v_per_rad_s{0.1};
    NonnegativeReal torque_constant_nm_per_a{0.1};
    PositiveReal inertia_kg_m2{0.03};
    NonnegativeReal viscous_friction_nm_per_rad_s{0.0};
    std::optional<bool> lock_rotor;
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
        return {{inverse_clarke_park(theta_e, state.id, state.iq)}, state.omega, state.theta};
    }

    MotorState derivative(const MotorState& state, const MotorInput& input) const {
        const auto& p = parameters_;
        const double omega_e = p.pole_pairs.get() * state.omega;
        const auto voltage = clarke_park(p.pole_pairs.get() * state.theta, input.voltage.abc);
        const double iq_dot = (voltage.y() - p.resistance_ohm.get() * state.iq -
                               omega_e * p.inductance_h.get() * state.id -
                               p.back_emf_v_per_rad_s.get() * state.omega) /
                              p.inductance_h.get();
        const double id_dot = (voltage.x() - p.resistance_ohm.get() * state.id +
                               omega_e * p.inductance_h.get() * state.iq) /
                              p.inductance_h.get();
        const double torque = 1.5 * p.torque_constant_nm_per_a.get() * state.iq;
        if (p.lock_rotor.value_or(false)) {
            return {iq_dot, id_dot, 0.0, 0.0};
        }
        const double omega_dot =
            (torque - input.load_torque - p.viscous_friction_nm_per_rad_s.get() * state.omega) /
            p.inertia_kg_m2.get();
        return {iq_dot, id_dot, omega_dot, state.omega};
    }

  private:
    MotorParameters parameters_;
};

namespace drive_blocks {

/// Applied phase voltages (V) and shaft load torque (N·m).
struct PlantInputs {
    InputPort<Voltage> va, vb, vc;
    InputPort<Torque> load_torque;
};

/// Motor phase currents (A), mechanical speed (rad/s), and position (rad).
struct PlantOutputs {
    Current ia, ib, ic;
    Speed omega;
    Position theta;
};

class ContinuousPmsm {
  public:
    static constexpr int state_size = 4;
    using State = Eigen::Vector<double, state_size>;

    explicit ContinuousPmsm(const BlockSpec& spec)
        : ContinuousPmsm(spec.read_parameters<MotorParameters>()) {}

    explicit ContinuousPmsm(const MotorParameters& parameters, MotorState initial = {})
        : plant_(parameters),
          initial_state_(initial.iq, initial.id,
                         parameters.lock_rotor.value_or(false) ? 0.0 : initial.omega,
                         initial.theta) {
        publish(initial_state_);
    }

    const State& initial_state() const { return initial_state_; }

    void publish(StateView state) {
        const auto out = plant_.output({state[0], state[1], state[2], state[3]});
        outputs = {out.current.abc[0], out.current.abc[1], out.current.abc[2], out.omega,
                   out.theta};
    }

    void derivative(StateView state, MutableStateView rate) const {
        const MotorInput input{{{inputs.va.read(), inputs.vb.read(), inputs.vc.read()}},
                               inputs.load_torque.read()};
        const auto out = plant_.derivative({state[0], state[1], state[2], state[3]}, input);
        rate << out.iq, out.id, out.omega, out.theta;
    }

    PlantInputs inputs;
    PlantOutputs outputs;

  private:
    PmsmPlant plant_;
    const State initial_state_;
};

} // namespace drive_blocks

} // namespace pmsm
