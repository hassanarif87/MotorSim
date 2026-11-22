#pragma once

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "blocks/plant.h"

namespace pmsm {

struct CurrentPiConfig {
    double kp;
    double ki;
    double kaw;
    double id_reference_a;
};

struct InverterConfig {
    double bus_voltage_v;
};

struct SpeedControlConfig {
    double gain_a_per_rad_s;
    double max_current_a;
    std::optional<double> reference_rad_s;
};

struct SimulationConfig {
    double load_torque_nm;
    double step_s;
    int steps;
    int log_every_steps;
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
    };
}

inline void from_json(const nlohmann::json& json, InverterConfig& config) {
    config.bus_voltage_v = config_detail::bounded_number(json, "bus_voltage_v", false);
}

inline void from_json(const nlohmann::json& json, SpeedControlConfig& config) {
    const auto reference = json.at("reference_rad_s").is_null()
        ? std::nullopt
        : std::optional<double>(config_detail::number(json, "reference_rad_s"));
    config = {
        config_detail::bounded_number(json, "gain_a_per_rad_s", true),
        config_detail::bounded_number(json, "max_current_a", true),
        reference,
    };
}

inline void from_json(const nlohmann::json& json, SimulationConfig& config) {
    config = {
        config_detail::number(json, "load_torque_nm"),
        config_detail::bounded_number(json, "step_s", false),
        config_detail::positive_integer(json, "steps"),
        config_detail::positive_integer(json, "log_every_steps"),
    };
}

inline void from_json(const nlohmann::json& json, Configuration& config) {
    json.at("motor").get_to(config.motor);
    json.at("current_pi").get_to(config.current_pi);
    json.at("inverter").get_to(config.inverter);
    json.at("speed_control").get_to(config.speed_control);
    json.at("simulation").get_to(config.simulation);
}

inline Configuration load_configuration(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Could not open configuration");
    }
    return nlohmann::json::parse(input).get<Configuration>();
}

}  // namespace pmsm
