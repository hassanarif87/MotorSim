#include <chrono>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "src/exec/simulation_executor.h"

namespace {

using Executor = pmsm::SimulationExecutor;
using namespace std::chrono_literals;

struct Trace {
    Executor& executor;
    std::vector<Executor::Duration> current_ticks;
    std::vector<char> events_at_zero;
    double input = 0.0;

    void speed() {
        if (executor.now() == 0ns)
            events_at_zero.push_back('s');
    }

    void current() {
        if (executor.now() == 0ns)
            events_at_zero.push_back('c');
        current_ticks.push_back(executor.now());
    }

    void pwm() {
        if (executor.now() == 0ns)
            events_at_zero.push_back('p');
        ++input;
    }
};

struct CurrentTick {
    Trace& trace;
    void tick() { trace.current(); }
};

struct WrongSignatures {
    void tick(int count = 0);
};

struct HeldInputBlock {
    static constexpr int state_size = 1;
    double published = 0.0;
    double input = 0.0;
    const double* external_input = nullptr;
    mutable int stages = 0;
    std::vector<double> samples;

    void publish(pmsm::StateView<HeldInputBlock> state) { published = state[0]; }

    void derivative(pmsm::StateView<HeldInputBlock>,
                    pmsm::MutableStateView<HeldInputBlock> rate) const {
        ++stages;
        rate[0] = external_input ? *external_input : input;
    }

    void tick() {
        samples.push_back(published);
        ++input;
    }
};

static_assert(pmsm::TickBlock<CurrentTick>);
static_assert(!pmsm::TickBlock<WrongSignatures>);

TEST(SimulationExecutorTest, RejectsUnalignedPeriod) {
    Executor executor(2ns);
    Trace trace{executor, {}, {}};
    CurrentTick current{trace};
    EXPECT_THROW(executor.register_block(current, 5ns, 1), std::invalid_argument);
}

TEST(SimulationExecutorTest, RunsCallbacksInOrderAndIntegratesPartialStep) {
    Executor executor(2ns);
    Trace trace{executor, {}, {}};
    CurrentTick current{trace};
    HeldInputBlock continuous;
    continuous.external_input = &trace.input;
    executor.register_block(continuous, Eigen::Vector<double, 1>::Zero());
    executor.register_tick(6ns, 2, Executor::TickCallback::bind<&Trace::pwm>(trace));
    executor.register_block(current, 4ns, 1);
    executor.register_tick(20ns, 0, Executor::TickCallback::bind<&Trace::speed>(trace));
    executor.run(13ns);

    EXPECT_EQ(trace.current_ticks, (std::vector<Executor::Duration>{0ns, 4ns, 8ns, 12ns}));
    EXPECT_EQ(trace.events_at_zero, (std::vector<char>{'s', 'c', 'p'}));
    EXPECT_EQ(executor.now(), 13ns);
    EXPECT_EQ(continuous.stages, 28);
    EXPECT_NEAR(continuous.published, 21e-9, 1e-15);
}

TEST(SimulationExecutorTest, PublishesInitialStateAndIntegratesHeldInputThroughPartialStep) {
    HeldInputBlock block;
    Executor executor(10us);
    Eigen::Vector<double, HeldInputBlock::state_size> initial_state;
    initial_state[0] = 1.0;
    executor.register_block(block, initial_state);
    executor.register_tick(50us, 0, Executor::TickCallback::bind<&HeldInputBlock::tick>(block));

    executor.run(55us);

    ASSERT_EQ(block.samples.size(), 2u);
    EXPECT_DOUBLE_EQ(block.samples[0], 1.0);
    EXPECT_NEAR(block.samples[1], 1.0 + 50e-6, 1e-12);
    EXPECT_NEAR(block.published, 1.0 + 60e-6, 1e-12);
    EXPECT_EQ(block.stages, 24);
    EXPECT_EQ(executor.now(), 55us);
    EXPECT_THROW(executor.register_block(block, initial_state), std::logic_error);
}

} // namespace
