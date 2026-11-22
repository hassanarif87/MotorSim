#pragma once

#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rfl/DefaultVal.hpp>
#include <rfl/Generic.hpp>
#include <rfl/Validator.hpp>
#include <rfl/comparisons.hpp>
#include <rfl/from_generic.hpp>

#include "src/graph/connections.h"
#include "src/simulation_executor.h"

namespace pmsm {

namespace config_value {

struct Finite {
    static rfl::Result<double> validate(double value) noexcept {
        if (!std::isfinite(value)) {
            return rfl::error("Expected a finite number");
        }
        return value;
    }
};

inline constexpr auto nanoseconds_per_second =
    std::chrono::duration_cast<SimulationExecutor::Duration>(std::chrono::seconds(1)).count();

struct ExactRate {
    static rfl::Result<int> validate(int rate) noexcept {
        if (rate <= 0 || nanoseconds_per_second % rate != 0) {
            return rfl::error("Rate must be positive and divide 1,000,000,000 Hz exactly");
        }
        return rate;
    }
};

}  // namespace config_value

using FiniteReal = rfl::Validator<double, config_value::Finite>;
using PositiveReal = rfl::Validator<double, config_value::Finite, rfl::ExclusiveMinimum<0>>;
using NonnegativeReal = rfl::Validator<double, config_value::Finite, rfl::Minimum<0>>;
using PositiveInt = rfl::Validator<int, rfl::Minimum<1>>;
using RateHz = rfl::Validator<int, config_value::ExactRate>;
using Priority = rfl::Validator<int, rfl::Minimum<0>,
                               rfl::ExclusiveMaximum<std::numeric_limits<int>::max()>>;

namespace config_value {

inline std::optional<SimulationExecutor::Duration> tick_period(
    const std::optional<RateHz>& rate, const std::optional<PositiveInt>& period) {
    using Duration = SimulationExecutor::Duration;
    if (rate) {
        return Duration(nanoseconds_per_second / rate->get());
    }
    return period ? std::optional{Duration(period->get())} : std::nullopt;
}

}  // namespace config_value

struct BlockSpec {
    std::string name;
    std::string type;
    rfl::DefaultVal<rfl::Generic::Object> parameters;
    std::optional<RateHz> rate_hz;
    std::optional<PositiveInt> period_ns;
    std::optional<Priority> priority;

    std::optional<SimulationExecutor::Duration> period() const {
        return config_value::tick_period(rate_hz, period_ns);
    }

    int tick_priority() const { return priority ? priority->get() : 0; }

    template <typename Parameters>
    Parameters read_parameters() const {
        return rfl::from_generic<Parameters>(rfl::Generic(parameters.get())).value();
    }
};

struct SimulationConfig {
    PositiveReal step_s{1e-5};
    PositiveInt steps{1};
    std::optional<PositiveInt> log_every_steps;
    std::optional<RateHz> log_rate_hz;
    std::optional<PositiveInt> log_period_ns;

    SimulationExecutor::Duration integration_step() const;
    SimulationExecutor::Duration logging_period() const;
};

struct Configuration {
    SimulationConfig simulation;
    std::vector<BlockSpec> blocks;
    std::optional<Connections> connections;
    std::optional<Connections> log_columns;
};

Configuration load_configuration(const std::filesystem::path& path);

}  // namespace pmsm
