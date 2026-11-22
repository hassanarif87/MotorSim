#pragma once

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "graph/connections.h"
#include "simulation_executor.h"

namespace pmsm {

struct BlockSpec {
    std::string name;
    std::string type;
    nlohmann::json parameters;
    std::optional<SimulationExecutor::Duration> period;
    int priority = 0;
};

struct SimulationConfig {
    double step_s;
    int steps;
    int log_every_steps;
    std::optional<SimulationExecutor::Duration> log_period;

    SimulationExecutor::Duration integration_step() const {
        using Duration = SimulationExecutor::Duration;
        const double nanoseconds = step_s * 1e9;
        const double rounded = std::round(nanoseconds);
        if (rounded < 1 || rounded >= static_cast<double>(Duration::max().count()) ||
            std::abs(nanoseconds - rounded) > 1e-6) {
            throw std::invalid_argument("step_s must represent a positive whole number of nanoseconds");
        }
        const auto step = Duration(static_cast<Duration::rep>(rounded));
        if (step > Duration::max() / steps ||
            (!log_period && step > Duration::max() / log_every_steps)) {
            throw std::invalid_argument("Simulation duration or logging period is out of range");
        }
        return step;
    }
};

struct Configuration {
    SimulationConfig simulation;
    std::vector<BlockSpec> blocks;
    std::optional<Connections> connections;
    std::optional<Connections> log_columns;
};

namespace config_detail {

using Json = nlohmann::json;

inline double number(const Json& section, const char* key) {
    const auto& value = section.at(key);
    if (!value.is_number() || !std::isfinite(value.get<double>())) {
        throw std::invalid_argument(std::string(key) + " must be a finite number");
    }
    return value.get<double>();
}

inline double bounded_number(const Json& section, const char* key, bool allow_zero) {
    const double value = number(section, key);
    if (value < 0.0 || (!allow_zero && value == 0.0)) {
        throw std::invalid_argument(std::string(key) +
                                    (allow_zero ? " must be nonnegative" : " must be positive"));
    }
    return value;
}

inline int positive_integer(const Json& section, const char* key) {
    const double value = number(section, key);
    if (!section.at(key).is_number_integer() || value < 1 ||
        value > std::numeric_limits<int>::max()) {
        throw std::invalid_argument(std::string(key) + " must be a positive integer in range");
    }
    return static_cast<int>(value);
}

inline int period_or(const Json& section, const char* key, int fallback) {
    return section.contains(key) ? positive_integer(section, key) : fallback;
}

inline std::optional<SimulationExecutor::Duration> tick_period(
    const Json& section, const char* rate_key, const char* legacy_period_key) {
    using Duration = SimulationExecutor::Duration;
    if (section.contains(rate_key)) {
        const int rate = positive_integer(section, rate_key);
        constexpr auto nanoseconds_per_second = std::chrono::duration_cast<Duration>(
            std::chrono::seconds(1)).count();
        if (nanoseconds_per_second % rate != 0) {
            throw std::invalid_argument(std::string(rate_key) +
                                        " must divide 1,000,000,000 Hz exactly");
        }
        return Duration(nanoseconds_per_second / rate);
    }
    if (section.contains(legacy_period_key)) {
        return Duration(positive_integer(section, legacy_period_key));
    }
    return std::nullopt;
}

}  // namespace config_detail

inline void from_json(const nlohmann::json& json, BlockSpec& block) {
    json.at("name").get_to(block.name);
    json.at("type").get_to(block.type);
    block.parameters = json.value("parameters", nlohmann::json::object());
    if (!block.parameters.is_object()) {
        throw std::invalid_argument("Block parameters must be an object: " + block.name);
    }
    block.period = config_detail::tick_period(json, "rate_hz", "period_ns");
    block.priority = 0;
    if (json.contains("priority")) {
        const double priority = config_detail::number(json, "priority");
        if (!json.at("priority").is_number_integer() || priority < 0 ||
            priority >= std::numeric_limits<int>::max()) {
            throw std::invalid_argument("Invalid block priority: " + block.name);
        }
        block.priority = static_cast<int>(priority);
    }
}

inline void from_json(const nlohmann::json& json, SimulationConfig& config) {
    config = {
        config_detail::bounded_number(json, "step_s", false),
        config_detail::positive_integer(json, "steps"),
        config_detail::period_or(json, "log_every_steps", 10),
        config_detail::tick_period(json, "log_rate_hz", "log_period_ns"),
    };
}

namespace config_detail {

inline std::optional<Connections> port_pairs(const Json& json, const char* key) {
    if (!json.contains(key)) {
        return std::nullopt;
    }
    const auto& entries = json.at(key);
    if (!entries.is_array()) {
        throw std::invalid_argument(std::string(key) + " must be an array of string pairs");
    }
    Connections pairs;
    for (const auto& entry : entries) {
        if (!entry.is_array() || entry.size() != 2 || !entry[0].is_string() || !entry[1].is_string()) {
            throw std::invalid_argument(std::string(key) + " requires pairs of strings");
        }
        pairs.push_back(entry.get<Connections::value_type>());
    }
    return pairs;
}

}  // namespace config_detail

inline void from_json(const nlohmann::json& json, Configuration& config) {
    json.at("simulation").get_to(config.simulation);
    config.simulation.integration_step();
    json.at("blocks").get_to(config.blocks);
    config.connections = config_detail::port_pairs(json, "connections");
    config.log_columns = config_detail::port_pairs(json, "log_columns");
}

inline Configuration load_configuration(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Could not open configuration");
    }
    return nlohmann::json::parse(input).get<Configuration>();
}

}  // namespace pmsm
