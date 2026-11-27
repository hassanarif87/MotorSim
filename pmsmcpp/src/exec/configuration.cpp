#include "src/exec/configuration.h"

#include <utility>

#include <rfl/json/load.hpp>

namespace pmsm {

namespace {

struct SimulationInput {
    RateHz tick_rate_hz;
    PositiveReal duration_s;
    RateHz log_rate_hz;
};

struct ConfigurationInput {
    SimulationInput simulation;
    std::vector<BlockSpec> blocks;
    Connections connections;
    Connections log_columns;
};

SimulationExecutor::Duration parse_duration(double seconds) {
    using Duration = SimulationExecutor::Duration;
    const double nanoseconds = seconds * config_value::nanoseconds_per_second;
    const double rounded = std::round(nanoseconds);
    if (rounded < 1 || rounded >= static_cast<double>(Duration::max().count()) ||
        std::abs(nanoseconds - rounded) > 1e-6) {
        throw std::invalid_argument(
            "duration_s must represent a positive whole number of nanoseconds");
    }
    return Duration(static_cast<Duration::rep>(rounded));
}

} // namespace

SimulationConfig::SimulationConfig(RateHz tick_rate_hz, PositiveReal duration_s, RateHz log_rate_hz)
    : tick_interval(config_value::period_from_rate(tick_rate_hz)),
      duration(parse_duration(duration_s.get())),
      logging_period(config_value::period_from_rate(log_rate_hz)) {
    if (logging_period % tick_interval != SimulationExecutor::Duration::zero()) {
        throw std::invalid_argument("Logging period must span a whole number of simulation ticks");
    }
}

Configuration load_configuration(const std::filesystem::path& path) {
    auto input = rfl::json::load<ConfigurationInput>(path.string()).value();
    const auto& sim = input.simulation;
    return {
        SimulationConfig(sim.tick_rate_hz, sim.duration_s, sim.log_rate_hz),
        std::move(input.blocks),
        std::move(input.connections),
        std::move(input.log_columns),
    };
}

} // namespace pmsm
