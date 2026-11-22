#pragma once

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "blocks/plant.h"
#include "simulation_executor.h"

namespace pmsm {

struct CurrentPiConfig {
    double kp;
    double ki;
    double kaw;
    double id_reference_a;
    SimulationExecutor::Duration period;
};

struct InverterConfig {
    double bus_voltage_v;
    SimulationExecutor::Duration pwm_period;
};

struct SpeedControlConfig {
    double gain_a_per_rad_s;
    double max_current_a;
    std::optional<double> reference_rad_s;
    SimulationExecutor::Duration period;
};

struct SimulationConfig {
    double load_torque_nm;
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
    MotorParameters motor;
    CurrentPiConfig current_pi;
    InverterConfig inverter;
    SpeedControlConfig speed_control;
    SimulationConfig simulation;
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

inline void from_json(const nlohmann::json& json, MotorParameters& motor) {
    using config_detail::bounded_number;
    motor = {
        bounded_number(json, "resistance_ohm", true),
        bounded_number(json, "inductance_h", false),
        config_detail::positive_integer(json, "pole_pairs"),
        bounded_number(json, "back_emf_v_per_rad_s", false),
        bounded_number(json, "torque_constant_nm_per_a", true),
        bounded_number(json, "inertia_kg_m2", false),
        bounded_number(json, "viscous_friction_nm_per_rad_s", true),
    };
}

inline void from_json(const nlohmann::json& json, CurrentPiConfig& config) {
    using config_detail::bounded_number;
    config = {
        bounded_number(json, "kp", true),
        bounded_number(json, "ki", true),
        bounded_number(json, "kaw", true),
        config_detail::number(json, "id_reference_a"),
        config_detail::tick_period(json, "rate_hz", "period_ns")
            .value_or(SimulationExecutor::Duration(50'000)),
    };
}

inline void from_json(const nlohmann::json& json, InverterConfig& config) {
    config = {config_detail::bounded_number(json, "bus_voltage_v", false),
              config_detail::tick_period(json, "pwm_rate_hz", "pwm_period_ns")
                  .value_or(SimulationExecutor::Duration(50'000))};
}

inline void from_json(const nlohmann::json& json, SpeedControlConfig& config) {
    const auto reference = json.at("reference_rad_s").is_null()
        ? std::nullopt
        : std::optional<double>(config_detail::number(json, "reference_rad_s"));
    config = {
        config_detail::bounded_number(json, "gain_a_per_rad_s", true),
        config_detail::bounded_number(json, "max_current_a", true),
        reference,
        config_detail::tick_period(json, "rate_hz", "period_ns")
            .value_or(SimulationExecutor::Duration(1'000'000)),
    };
}

inline void from_json(const nlohmann::json& json, SimulationConfig& config) {
    config = {
        config_detail::number(json, "load_torque_nm"),
        config_detail::bounded_number(json, "step_s", false),
        config_detail::positive_integer(json, "steps"),
        config_detail::period_or(json, "log_every_steps", 10),
        config_detail::tick_period(json, "log_rate_hz", "log_period_ns"),
    };
}

inline void from_json(const nlohmann::json& json, Configuration& config) {
    json.at("motor").get_to(config.motor);
    json.at("current_pi").get_to(config.current_pi);
    json.at("inverter").get_to(config.inverter);
    json.at("speed_control").get_to(config.speed_control);
    json.at("simulation").get_to(config.simulation);
    config.simulation.integration_step();
}

inline Configuration load_configuration(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Could not open configuration");
    }
    return nlohmann::json::parse(input).get<Configuration>();
}

}  // namespace pmsm
