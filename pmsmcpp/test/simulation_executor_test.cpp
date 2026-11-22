#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "src/rk4.h"
#include "src/simulation_executor.h"

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
        if (executor.now() == 0ns) events_at_zero.push_back('s');
    }

    void current() {
        if (executor.now() == 0ns) events_at_zero.push_back('c');
        current_ticks.push_back(executor.now());
        output_ready = output_ready && std::abs(published - state) < 1e-12;
    }

    void pwm() {
        if (executor.now() == 0ns) events_at_zero.push_back('p');
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

}  // namespace

int main() {
    Executor executor(4ns);
    Trace trace{executor, {}, {}, {}};
    CurrentTick current{trace};
    executor.register_block(trace);
    executor.register_update(7ns, 2, Executor::UpdateCallback::bind<&Trace::pwm>(trace));
    executor.register_block(current, 5ns, 1);
    executor.register_update(20ns, 0, Executor::UpdateCallback::bind<&Trace::speed>(trace));
    executor.run(12ns);

    if (trace.intervals != std::vector<Executor::Duration>{4ns, 1ns, 2ns, 3ns, 2ns} ||
        trace.current_ticks != std::vector<Executor::Duration>{0ns, 5ns, 10ns} ||
        trace.events_at_zero != std::vector<char>{'s', 'c', 'p'} ||
        trace.stages != 20 || !trace.output_ready || std::abs(trace.state - 17.0) > 1e-12) {
        std::cerr << "Unexpected callback order, integration steps, or held-input result\n";
        return 1;
    }
}
