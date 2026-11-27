#include <cmath>
#include <string>

#include "gtest/gtest.h"

#include "src/blocks/inverter.h"
#include "src/blocks/continuous_pmsm.h"
#include "src/blocks/simplefoc.h"
#include "src/blocks/svpwm.h"

namespace {

TEST(BlocksTest, PlantDynamicsAndOutput) {
    const pmsm::PmsmPlant plant({0.5, 0.25, 2, 0.2, 0.2, 0.1, 0.01});
    const pmsm::MotorState state{3.0, 4.0, 5.0, std::atan(1.0)};
    const Eigen::Vector3d abc(0.0, std::sqrt(3.0), -std::sqrt(3.0));
    const auto rate = plant.derivative(state, {{abc}, 0.1});
    EXPECT_NEAR(rate.iq, -50.0, 1e-10) << "iq derivative";
    EXPECT_NEAR(rate.id, 30.0, 1e-10) << "id derivative";
    EXPECT_NEAR(rate.omega, 7.5, 1e-10) << "omega derivative";
    EXPECT_NEAR(rate.theta, 5.0, 1e-10) << "theta derivative";

    const auto shifted = plant.derivative(state, {{abc.array() + 24.0}, 0.1});
    EXPECT_NEAR(shifted.iq, rate.iq, 1e-10) << "iq derivative with common mode";
    EXPECT_NEAR(shifted.id, rate.id, 1e-10) << "id derivative with common mode";

    const auto output = plant.output(state);
    EXPECT_NEAR(output.current.abc[0], -3.0, 1e-10) << "phase A current";
    EXPECT_NEAR(output.current.abc[1], 1.5 + 2.0 * std::sqrt(3.0), 1e-10) << "phase B current";
    EXPECT_NEAR(output.current.abc.sum(), 0.0, 1e-10) << "phase current sum";
}

TEST(BlocksTest, SimpleFocCurrentControl) {
    using namespace pmsm::drive_blocks;
    rfl::Generic::Object pid;
    pid.insert(std::string("P"), rfl::Generic(2.0));
    pid.insert(std::string("I"), rfl::Generic(4.0));
    pid.insert(std::string("D"), rfl::Generic(0.0));
    pid.insert(std::string("output_ramp"), rfl::Generic(0.0));
    rfl::Generic::Object parameters;
    parameters.insert(std::string("PID_current_d"), rfl::Generic(pid));
    parameters.insert(std::string("PID_current_q"), rfl::Generic(pid));
    parameters.insert(std::string("LPF_current_d_Tf"), rfl::Generic(0.1));
    parameters.insert(std::string("LPF_current_q_Tf"), rfl::Generic(0.1));
    parameters.insert(std::string("voltage_limit"), rfl::Generic(5.0));
    parameters.insert(std::string("feedforward_enabled"), rfl::Generic(true));
    parameters.insert(std::string("pole_pairs"), rfl::Generic(2));
    parameters.insert(std::string("inductance_h"), rfl::Generic(0.125));
    parameters.insert(std::string("back_emf_v_per_rad_s"), rfl::Generic(0.2));
    SimpleFocCurrent controller(pmsm::BlockSpec{
        .name = "current_pi",
        .type = "SimpleFocCurrent",
        .parameters = parameters,
        .rate_hz = pmsm::RateHz{10},
    });
    Current iq_ref{3.0};
    const Current id{2.0}, iq{4.0};
    const Speed omega{1.0};
    controller.inputs.iq_ref.bind(iq_ref);
    controller.inputs.id.bind(id);
    controller.inputs.iq.bind(iq);
    controller.inputs.omega.bind(omega);

    controller.tick();
    EXPECT_NEAR(controller.outputs.id_filtered, 1.0, 1e-10) << "filtered d current";
    EXPECT_NEAR(controller.outputs.iq_filtered, 2.0, 1e-10) << "filtered q current";
    EXPECT_NEAR(controller.outputs.vd, -3.2, 1e-10) << "first SimpleFOC d voltage";
    EXPECT_NEAR(controller.outputs.vq, 2.9, 1e-10) << "first SimpleFOC q voltage";

    controller.tick();
    EXPECT_NEAR(controller.outputs.vd, -4.7, 1e-10) << "second SimpleFOC d voltage";
    EXPECT_NEAR(controller.outputs.vq, 1.1, 1e-10) << "second SimpleFOC q voltage";

    iq_ref = 20.0;
    controller.tick();
    EXPECT_NEAR(controller.outputs.vd, -5.0, 1e-10) << "limited SimpleFOC d voltage";
    EXPECT_NEAR(controller.outputs.vq, 5.0, 1e-10) << "limited SimpleFOC q voltage";
}

TEST(BlocksTest, ReconstructsVoltageThroughIdealInverter) {
    const auto duty = pmsm::svpwm({0.0, 1.0}, 0.0, std::sqrt(3.0));
    const auto voltage = pmsm::drive_blocks::Inverter::ideal_inverter(duty, std::sqrt(3.0));
    const auto dq = pmsm::clarke_park(0.0, voltage.abc);
    EXPECT_NEAR(dq.x(), 0.0, 1e-10) << "reconstructed d voltage";
    EXPECT_NEAR(dq.y(), 1.0, 1e-10) << "reconstructed q voltage";
}

TEST(BlocksTest, NonIdealInverterLatchingAndVoltageDrop) {
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
    EXPECT_NEAR(inverter.outputs.va, 0.0, 1e-10) << "initial latched phase A";
    a = 1.2;
    b = -0.2;
    ia = 2.0;
    ib = -2.0;
    inverter.tick();
    EXPECT_NEAR(inverter.outputs.va, 5.72, 1e-10) << "latched phase A with voltage drop";
    EXPECT_NEAR(inverter.outputs.vb, -5.72, 1e-10) << "latched phase B with voltage drop";
    EXPECT_NEAR(inverter.outputs.vc, 0.0, 1e-10) << "latched phase C";
    inverter.tick();
    EXPECT_NEAR(inverter.outputs.va, 11.96, 1e-10) << "clipped phase A without dead time";
    EXPECT_NEAR(inverter.outputs.vb, -11.96, 1e-10) << "clipped phase B without dead time";
}

} // namespace
