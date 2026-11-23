#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "src/blocks/current_pi.h"
#include "src/blocks/inverter.h"
#include "src/blocks/continuous_pmsm.h"
#include "src/blocks/svpwm.h"

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
    using namespace pmsm::drive_blocks;
    rfl::Generic::Object parameters;
    parameters.insert(std::string("kp"), rfl::Generic(2.0));
    parameters.insert(std::string("ki"), rfl::Generic(1.0));
    parameters.insert(std::string("kaw"), rfl::Generic(2.0));
    parameters.insert(std::string("inductance_h"), rfl::Generic(0.25));
    parameters.insert(std::string("pole_pairs"), rfl::Generic(2));
    parameters.insert(std::string("back_emf_v_per_rad_s"), rfl::Generic(0.2));
    parameters.insert(std::string("id_reference_a"), rfl::Generic(0.0));
    parameters.insert(std::string("bus_voltage_v"), rfl::Generic(std::sqrt(3.0)));
    const pmsm::BlockSpec spec{
        .name = "current_pi",
        .type = "CurrentController",
        .parameters = parameters,
        .rate_hz = pmsm::RateHz{10},
    };
    CurrentController controller(spec);
    Current iq_ref{10.0};
    const Current measured{0.0};
    const Speed omega{0.0};
    controller.inputs.iq_ref.bind(iq_ref);
    controller.inputs.id.bind(measured);
    controller.inputs.iq.bind(measured);
    controller.inputs.omega.bind(omega);

    controller.tick();
    expect_near(controller.outputs.vq, 1.0, "limited q voltage");
    const auto duty = pmsm::svpwm({controller.outputs.vd, controller.outputs.vq},
                                 0.0, std::sqrt(3.0));

    iq_ref = 1.0;
    controller.tick();
    expect_near(controller.outputs.vq, -0.8, "q voltage after anti-windup");

    const auto voltage = pmsm::ideal_inverter(duty, std::sqrt(3.0));
    const auto dq = pmsm::clarke_park(0.0, voltage.abc);
    expect_near(dq.x(), 0.0, "reconstructed d voltage");
    expect_near(dq.y(), 1.0, "reconstructed q voltage");
}

void check_nonideal_inverter() {
    using namespace pmsm::drive_blocks;
    rfl::Generic::Object parameters;
    parameters.insert(std::string("bus_voltage_v"), rfl::Generic(24.0));
    parameters.insert(std::string("dead_time_s"), rfl::Generic(0.0000005));
    parameters.insert(std::string("switch_resistance_ohm"), rfl::Generic(0.02));
    NonIdealInverter inverter(pmsm::BlockSpec{
        .name = "inverter",
        .type = "NonIdealInverter",
        .parameters = parameters,
        .rate_hz = pmsm::RateHz{20000},
    });
    Duty a{0.75}, b{0.25}, c{0.5};
    Current ia{0.0}, ib{0.0}, ic{0.0};
    inverter.inputs.duty_a.bind(a);
    inverter.inputs.duty_b.bind(b);
    inverter.inputs.duty_c.bind(c);
    inverter.inputs.ia.bind(ia);
    inverter.inputs.ib.bind(ib);
    inverter.inputs.ic.bind(ic);

    inverter.tick();
    expect_near(inverter.outputs.va, 0.0, "initial latched phase A");
    a = 1.2;
    b = -0.2;
    ia = 2.0;
    ib = -2.0;
    inverter.tick();
    expect_near(inverter.outputs.va, 5.72, "latched phase A with voltage drop");
    expect_near(inverter.outputs.vb, -5.72, "latched phase B with voltage drop");
    expect_near(inverter.outputs.vc, 0.0, "latched phase C");
    inverter.tick();
    expect_near(inverter.outputs.va, 11.96, "clipped phase A without dead time");
    expect_near(inverter.outputs.vb, -11.96, "clipped phase B without dead time");
}

}  // namespace

int main() {
    try {
        check_plant();
        check_voltage_blocks();
        check_nonideal_inverter();
    } catch (const std::runtime_error&) {
        return 1;
    }
}
