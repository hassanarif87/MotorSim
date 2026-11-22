#include "src/configuration.h"

#include <rfl/json/load.hpp>

namespace pmsm {

namespace {

SimulationExecutor::Duration whole_nanoseconds(double seconds, const char* field) {
    using Duration = SimulationExecutor::Duration;
    const double nanoseconds = seconds * config_value::nanoseconds_per_second;
    const double rounded = std::round(nanoseconds);
    if (rounded < 1 || rounded >= static_cast<double>(Duration::max().count()) ||
        std::abs(nanoseconds - rounded) > 1e-6) {
        throw std::invalid_argument(std::string(field) + " must represent a positive whole number of nanoseconds");
    }
    return Duration(static_cast<Duration::rep>(rounded));
}

}  // namespace

SimulationExecutor::Duration SimulationConfig::tick_interval() const {
    if (tick_rate_hz) {
        return SimulationExecutor::Duration(config_value::nanoseconds_per_second / tick_rate_hz->get());
    }
    return whole_nanoseconds(step_s ? step_s->get() : 1e-5, "step_s");
}

SimulationExecutor::Duration SimulationConfig::duration() const {
    if (duration_s) {
        return whole_nanoseconds(duration_s->get(), "duration_s");
    }
    const auto step = tick_interval();
    const auto step_count = steps ? steps->get() : 1;
    if (step > SimulationExecutor::Duration::max() / step_count) {
        throw std::invalid_argument("Simulation duration is out of range");
    }
    return step * step_count;
}

SimulationExecutor::Duration SimulationConfig::logging_period() const {
    if (const auto period = config_value::tick_period(log_rate_hz, log_period_ns)) {
        return *period;
    }
    const int stride = log_every_steps ? log_every_steps->get() : 10;
    const auto step = tick_interval();
    if (step > SimulationExecutor::Duration::max() / stride) {
        throw std::invalid_argument("Logging period is out of range");
    }
    return step * stride;
}

Configuration load_configuration(const std::filesystem::path& path) {
    auto config = rfl::json::load<Configuration>(path.string()).value();
    const auto tick_interval = config.simulation.tick_interval();
    config.simulation.duration();
    if (config.simulation.logging_period() % tick_interval != SimulationExecutor::Duration::zero()) {
        throw std::invalid_argument("Logging period must span a whole number of simulation ticks");
    }
    return config;
}

}  // namespace pmsm
