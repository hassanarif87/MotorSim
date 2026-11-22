#include <cmath>
#include <iostream>
#include <stdexcept>

#include "blocks/current_pi.h"
#include "blocks/inverter.h"
#include "blocks/plant.h"
#include "blocks/svpwm.h"

namespace {

void expect_near(double actual, double expected, const char* signal) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-10) {
        std::cerr << signal << ": got " << actual << ", expected " << expected << '\n';
        throw std::runtime_error("Block result differs from reference");
    }
}

void check_plant() {
    const pmsm::PmsmPlant plant({0.5, 0.25, 2, 0.2, 0.2, 0.1, 0.01});
    const pmsm::MotorState state{3.0, 4.0, 5.0, std::atan(1.0)};
    const Eigen::Vector3d abc(0.0, std::sqrt(3.0), -std::sqrt(3.0));
    const auto rate = plant.derivative(state, {{abc}, 0.1});
    expect_near(rate.iq, -50.0, "iq derivative");
    expect_near(rate.id, 30.0, "id derivative");
    expect_near(rate.omega, 7.5, "omega derivative");
    expect_near(rate.theta, 5.0, "theta derivative");

    const auto shifted = plant.derivative(state, {{abc.array() + 24.0}, 0.1});
    expect_near(shifted.iq, rate.iq, "iq derivative with common mode");
    expect_near(shifted.id, rate.id, "id derivative with common mode");

    const auto output = plant.output(state);
    expect_near(output.current.abc[0], -3.0, "phase A current");
    expect_near(output.current.abc[1], 1.5 + 2.0 * std::sqrt(3.0), "phase B current");
    expect_near(output.current.abc.sum(), 0.0, "phase current sum");
}

void check_voltage_blocks() {
    const pmsm::CurrentPi pi({2.0, 1.0, 2.0, 0.25, 2, 0.2});
    const auto control = pi.evaluate(Eigen::Vector2d::Zero(),
                                     {{0.0, 10.0}, {0.0, 0.0}, 0.0, std::sqrt(3.0)});
    expect_near(control.voltage.q, 1.0, "limited q voltage");
    expect_near(control.integral_derivative.y(), -28.0, "q integrator anti-windup");

    const auto duty = pmsm::svpwm(control.voltage, 0.0, std::sqrt(3.0));
    const auto voltage = pmsm::ideal_inverter(duty, std::sqrt(3.0));
    const auto dq = pmsm::clarke_park(0.0, voltage.abc);
    expect_near(dq.x(), 0.0, "reconstructed d voltage");
    expect_near(dq.y(), 1.0, "reconstructed q voltage");
}

}  // namespace

int main() {
    try {
        check_plant();
        check_voltage_blocks();
    } catch (const std::runtime_error&) {
        return 1;
    }
}
