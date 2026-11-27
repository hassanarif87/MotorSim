#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "src/math/rk4.h"
#include "src/exec/simulation_executor.h"

namespace {

using Executor = pmsm::SimulationExecutor;
using namespace std::chrono_literals;

struct Trace {
    Executor& executor;
    std::vector<Executor::Duration> intervals;
    std::vector<Executor::Duration> current_ticks;
    std::vector<char> events_at_zero;
    int stages = 0;
    double state = 0;
    double published = 0;
    int input = 0;
    bool output_ready = true;

    void advance(Executor::Duration dt) {
        intervals.push_back(dt);
        state = pmsm::rk4_step(state, static_cast<double>(dt.count()), [this](double) {
            ++stages;
            return static_cast<double>(input);
        });
        published = state;
    }

    void speed() {
        if (executor.now() == 0ns)
            events_at_zero.push_back('s');
    }

    void current() {
        if (executor.now() == 0ns)
            events_at_zero.push_back('c');
        current_ticks.push_back(executor.now());
        output_ready = output_ready && std::abs(published - state) < 1e-12;
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
    void advance(double seconds);
};

static_assert(pmsm::TickBlock<CurrentTick>);
static_assert(pmsm::AdvanceBlock<Trace, Executor::Duration>);
static_assert(!pmsm::TickBlock<WrongSignatures>);
static_assert(!pmsm::AdvanceBlock<WrongSignatures, Executor::Duration>);

} // namespace

int main() {
    Executor executor(2ns);
    Trace trace{executor, {}, {}, {}};
    CurrentTick current{trace};
    executor.register_block(trace);
    try {
        executor.register_block(current, 5ns, 1);
        std::cerr << "Expected an unaligned period to be rejected\n";
        return 1;
    } catch (const std::invalid_argument&) {
    }
    executor.register_tick(6ns, 2, Executor::TickCallback::bind<&Trace::pwm>(trace));
    executor.register_block(current, 4ns, 1);
    executor.register_tick(20ns, 0, Executor::TickCallback::bind<&Trace::speed>(trace));
    executor.run(13ns);

    if (trace.intervals != std::vector<Executor::Duration>{2ns, 2ns, 2ns, 2ns, 2ns, 2ns, 1ns} ||
        trace.current_ticks != std::vector<Executor::Duration>{0ns, 4ns, 8ns, 12ns} ||
        trace.events_at_zero != std::vector<char>{'s', 'c', 'p'} || executor.now() != 13ns ||
        trace.stages != 28 || !trace.output_ready || std::abs(trace.state - 21.0) > 1e-12) {
        std::cerr << "Unexpected callback order, integration steps, or held-input result\n";
        return 1;
    }
}
