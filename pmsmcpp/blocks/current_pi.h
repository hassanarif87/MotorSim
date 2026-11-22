#pragma once

#include <algorithm>
#include <cmath>

#include "blocks/signals.h"

namespace pmsm {

struct CurrentPiParameters {
    double kp;
    double ki;
    double kaw;
    double inductance;
    int pole_pairs;
    double back_emf_constant;
};

struct CurrentPiInput {
    DqCurrent reference;
    DqCurrent measured;
    double omega;
    double bus_voltage;
};

struct CurrentPiEvaluation {
    DqVoltage voltage;
    Eigen::Vector2d integral_derivative;
};

class CurrentPi {
public:
    explicit CurrentPi(CurrentPiParameters parameters) : parameters_(parameters) {}

    CurrentPiEvaluation evaluate(const Eigen::Vector2d& integral,
                                 const CurrentPiInput& input) const {
        const auto& p = parameters_;
        const Eigen::Vector2d error(input.reference.d - input.measured.d,
                                    input.reference.q - input.measured.q);
        const double coupling = p.pole_pairs * input.omega * p.inductance;
        const Eigen::Vector2d feedforward(-coupling * input.measured.q,
            coupling * input.measured.d + p.back_emf_constant * input.omega);
        const Eigen::Vector2d raw = p.kp * error + integral + feedforward;
        const double limit = input.bus_voltage / std::sqrt(3.0);
        const Eigen::Vector2d voltage = raw * std::min(1.0, limit / std::max(raw.norm(), 1e-12));
        return {{voltage.x(), voltage.y()}, p.ki * error + p.kaw * (voltage - raw)};
    }

private:
    CurrentPiParameters parameters_;
};

}  // namespace pmsm
