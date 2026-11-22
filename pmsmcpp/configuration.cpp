#include "configuration.h"

#include <rfl/json/load.hpp>

namespace pmsm {

SimulationExecutor::Duration SimulationConfig::integration_step() const {
    using Duration = SimulationExecutor::Duration;
    const double nanoseconds = step_s.get() * config_value::nanoseconds_per_second;
    const double rounded = std::round(nanoseconds);
    if (rounded < 1 || rounded >= static_cast<double>(Duration::max().count()) ||
        std::abs(nanoseconds - rounded) > 1e-6) {
        throw std::invalid_argument("step_s must represent a positive whole number of nanoseconds");
    }
    const auto step = Duration(static_cast<Duration::rep>(rounded));
    if (step > Duration::max() / steps.get()) {
        throw std::invalid_argument("Simulation duration is out of range");
    }
    return step;
}

SimulationExecutor::Duration SimulationConfig::logging_period() const {
    if (const auto period = config_value::tick_period(log_rate_hz, log_period_ns)) {
        return *period;
    }
    const int stride = log_every_steps ? log_every_steps->get() : 10;
    const auto step = integration_step();
    if (step > SimulationExecutor::Duration::max() / stride) {
        throw std::invalid_argument("Logging period is out of range");
    }
    return step * stride;
}

Configuration load_configuration(const std::filesystem::path& path) {
    auto config = rfl::json::load<Configuration>(path.string()).value();
    config.simulation.integration_step();
    config.simulation.logging_period();
    return config;
}

}  // namespace pmsm
