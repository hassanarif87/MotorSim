#pragma once

#include "blocks/transforms.h"

namespace pmsm {

struct MotorParameters {
    double resistance = 0.03;
    double inductance = 0.03;
    int pole_pairs = 1;
    double back_emf_constant = 0.1;
    double torque_constant = 0.1;
    double inertia = 0.03;
    double friction = 0.0;
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

    int pole_pairs() const { return parameters_.pole_pairs; }

    MotorOutput output(const MotorState& state) const {
        const double theta_e = parameters_.pole_pairs * state.theta;
        return {{inverse_clarke_park(theta_e, state.id, state.iq)},
                state.omega, state.theta};
    }

    MotorState derivative(const MotorState& state, const MotorInput& input) const {
        const auto& p = parameters_;
        const double omega_e = p.pole_pairs * state.omega;
        const auto voltage = clarke_park(p.pole_pairs * state.theta, input.voltage.abc);
        const double iq_dot =
            (voltage.y() - p.resistance * state.iq - omega_e * p.inductance * state.id
             - p.back_emf_constant * state.omega) / p.inductance;
        const double id_dot =
            (voltage.x() - p.resistance * state.id + omega_e * p.inductance * state.iq)
            / p.inductance;
        const double torque = 1.5 * p.torque_constant * state.iq;
        const double omega_dot =
            (torque - input.load_torque - p.friction * state.omega) / p.inertia;
        return {iq_dot, id_dot, omega_dot, state.omega};
    }

private:
    MotorParameters parameters_;
};

}  // namespace pmsm
