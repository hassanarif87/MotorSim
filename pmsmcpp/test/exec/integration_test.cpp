#include <chrono>
#include <cmath>

#include <Eigen/Core>

#include "gtest/gtest.h"

#include "src/exec/integration.h"

namespace pmsm {
namespace {

class SpringMassDamper {

  public:
    static constexpr int state_size = 2;
    using State = StateView;
    using StateDot = MutableStateView;

    double position = 0.0;
    double velocity = 0.0;

    void publish(State state) {
        position = state[0];
        velocity = state[1];
    }

    void derivative(State state, StateDot state_dot) const {
        state_dot[0] = state[1];
        state_dot[1] = (-spring_constant * state[0] - damping_coefficient * state[1]) / mass;
    }

  private:
    double spring_constant = 1.0;
    double damping_coefficient = 1.0;
    double mass = 1.0;
};

static_assert(ContinousBlock<SpringMassDamper>);

struct SpringMassDamperState {
    double position;
    double velocity;
};

SpringMassDamperState analytical_state(double time, double initial_position,
                                       double initial_velocity) {
    // For m = c = k = 1, x'' + x' + x = 0.
    // x(t) = exp(-alpha*t) * (A*cos(omega*t) + B*sin(omega*t)).
    constexpr double alpha = 0.5;
    const double omega = std::sqrt(1.0 - alpha * alpha);
    const double cosine_amplitude = initial_position;
    const double sine_amplitude = (initial_velocity + alpha * initial_position) / omega;
    const double decay = std::exp(-alpha * time);
    const double cosine = std::cos(omega * time);
    const double sine = std::sin(omega * time);
    
    const double position = decay * (cosine_amplitude * cosine + sine_amplitude * sine);
    const double velocity =
        decay * omega * (-cosine_amplitude * sine + sine_amplitude * cosine) - alpha * position;
    return {position, velocity};
}

TEST(IntegrationTest, MatchesAnalyticalSpringMassDamperSolution) {
    SpringMassDamper spring_mass_damper;

    constexpr double initial_position = 1.0;
    constexpr double initial_velocity = 0.5;
    Eigen::Vector2d initial_state{initial_position, initial_velocity};
    IntegrationExecutor integration;
    integration.add(spring_mass_damper, initial_state);
    integration.initialize();

    EXPECT_EQ(spring_mass_damper.position, initial_position);
    EXPECT_EQ(spring_mass_damper.velocity, initial_velocity);

    constexpr auto duration = std::chrono::seconds{1};
    constexpr auto step = std::chrono::milliseconds{10};

    for (auto elapsed = step; elapsed <= duration; elapsed += step) {
        integration.advance(step);

        const double time = std::chrono::duration<double>(elapsed).count();
        SCOPED_TRACE(::testing::Message() << "t = " << time << " s");
        const auto [expected_position, expected_velocity] =
            analytical_state(time, initial_position, initial_velocity);

        EXPECT_NEAR(spring_mass_damper.position, expected_position, 1e-8);
        EXPECT_NEAR(spring_mass_damper.velocity, expected_velocity, 1e-8);
    }
}

} // namespace
} // namespace pmsm
