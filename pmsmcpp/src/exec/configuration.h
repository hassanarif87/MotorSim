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
#include "src/exec/simulation_executor.h"

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

/// @brief Nanoseconds per second constant for rate calculations.
inline constexpr auto nanoseconds_per_second =
    std::chrono::duration_cast<SimulationExecutor::Duration>(std::chrono::seconds(1)).count();

/// @brief Validator for rates.
struct ExactRate {
    static rfl::Result<int> validate(int rate) noexcept {
        if (rate <= 0 || nanoseconds_per_second % rate != 0) {
            return rfl::error("Rate must be positive and divide 1,000,000,000 Hz exactly");
        }
        return rate;
    }
};

} // namespace config_value

/// @brief Type aliases for configuration values with validation rules.
using FiniteReal = rfl::Validator<double, config_value::Finite>;
using PositiveReal = rfl::Validator<double, config_value::Finite, rfl::ExclusiveMinimum<0>>;
using NonnegativeReal = rfl::Validator<double, config_value::Finite, rfl::Minimum<0>>;
using PositiveInt = rfl::Validator<int, rfl::Minimum<1>>;
using RateHz = rfl::Validator<int, config_value::ExactRate>;
using Priority =
    rfl::Validator<int, rfl::Minimum<0>, rfl::ExclusiveMaximum<std::numeric_limits<int>::max()>>;

namespace config_value {

/// @brief Calculates the period for a given rate.
/// @param rate The rate in Hz for which to calculate the period. Comes from the config.
/// @return Duration representing the period in nanoseconds corresponding to the given rate.
inline SimulationExecutor::Duration period_from_rate(RateHz rate) {
    return SimulationExecutor::Duration(nanoseconds_per_second / rate.get());
}

} // namespace config_value

/// @brief Represents the specification of a block in the simulation.
struct BlockSpec {
    std::string name;
    std::string type;
    rfl::DefaultVal<rfl::Generic::Object> parameters;
    std::optional<RateHz> rate_hz;
    std::optional<Priority> priority;

    std::optional<SimulationExecutor::Duration> period() const {
        if (rate_hz) {
            return config_value::period_from_rate(*rate_hz);
        }
        return std::nullopt;
    }

    int tick_priority() const { return priority ? priority->get() : 0; }

    template <typename Parameters>
    Parameters read_parameters() const {
        return rfl::from_generic<Parameters>(rfl::Generic(parameters.get())).value();
    }
};

/// @brief Represents the top-level simulation parameters.
struct SimulationConfig {
    SimulationConfig(RateHz tick_rate_hz, PositiveReal duration_s, RateHz log_rate_hz);

    SimulationExecutor::Duration tick_interval;
    SimulationExecutor::Duration duration;
    SimulationExecutor::Duration logging_period;
};

/// @brief Represents the entire configuration for a simulation.
struct Configuration {
    SimulationConfig simulation;
    std::vector<BlockSpec> blocks;
    Connections connections;
    Connections log_columns;
};

/// @brief Load a configuration from a JSON file, config file contains:
///
/// - Executable simulation parameters (tick rate, duration, log rate)
///
/// - Block specifications (name, type, parameters, optional rate and priority)
/// @param path Path to the JSON config file
/// @return Loaded configuration
Configuration load_configuration(const std::filesystem::path& path);

} // namespace pmsm
