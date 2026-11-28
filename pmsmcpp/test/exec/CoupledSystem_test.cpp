#include <array>
#include <chrono>

#include <Eigen/Core>

#include "gtest/gtest.h"

#include "src/exec/integration.h"
#include "src/graph/block_ports.h"

namespace pmsm {
namespace {

struct SpringMass {
    static constexpr int state_size = 2;

    InputPort<double> neighbor_position;
    double position = 0.0;
    double velocity = 0.0;

    void publish(StateView state) {
        position = state[0];
        velocity = state[1];
    }

    void derivative(StateView state, MutableStateView rate) const {
        rate[0] = state[1];
        rate[1] = neighbor_position.read() - state[0];
    }
};

TEST(CoupledSystemTest, IntegratesSpringCoupledMassesInEitherRegistrationOrder) {
    for (const bool reverse_order : std::array{false, true}) {
        SCOPED_TRACE(::testing::Message() << "reverse registration order: " << reverse_order);
        SpringMass left;
        SpringMass right;
        left.neighbor_position.bind(right.position);
        right.neighbor_position.bind(left.position);

        Eigen::Vector2d left_initial{-1.0, 0.0};
        Eigen::Vector2d right_initial{1.0, 0.0};
        IntegrationExecutor integration;
        if (reverse_order) {
            integration.add(right, right_initial);
            integration.add(left, left_initial);
        } else {
            integration.add(left, left_initial);
            integration.add(right, right_initial);
        }
        integration.initialize();

        constexpr auto interval = std::chrono::milliseconds{500};
        constexpr int step_count = 10;

        for (int step = 1; step <= step_count; ++step) {
            SCOPED_TRACE(::testing::Message() << "integration step " << step);
            integration.advance(interval);

            EXPECT_NEAR(left.position, -right.position, 1e-12);
            EXPECT_NEAR(left.velocity, -right.velocity, 1e-12);
            if (step == 1) {
                // One RK4 step of x'' = -2x from x = 1 and v = 0.
                EXPECT_NEAR(right.position, 73.0 / 96.0, 1e-12);
                EXPECT_NEAR(right.velocity, -11.0 / 12.0, 1e-12);
            }
        }
    }
}

} // namespace
} // namespace pmsm
