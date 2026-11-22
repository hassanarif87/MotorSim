#include <array>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <csv.hpp>

#include "blocks/speed_control.h"
#include "configuration.h"
#include "drive.h"
#include "rk4.h"

namespace {

using State = pmsm::CurrentControlledPmsm::State;

// Give the CSV library a round-trippable field representation.
struct CsvNumber {
    CsvNumber(double field) : value(field) {}

    double value;

    operator std::string() const {
        char buffer[64];
        const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value,
            std::chars_format::general, std::numeric_limits<double>::max_digits10);
        if (result.ec != std::errc{}) {
            throw std::runtime_error("Could not format CSV number");
        }
        return {buffer, result.ptr};
    }
};

void simulate(std::ostream& output, const pmsm::Configuration& config) {
    const auto& motor = config.motor;
    const auto& pi = config.current_pi;
    const auto& sim = config.simulation;
    const auto& speed_config = config.speed_control;
    const double bus_voltage = config.inverter.bus_voltage_v;
    const pmsm::CurrentPi controller({
        pi.kp, pi.ki, pi.kaw, motor.inductance, motor.pole_pairs, motor.back_emf_constant,
    });
    const pmsm::CurrentControlledPmsm drive(pmsm::PmsmPlant(motor), controller);
    const pmsm::SpeedControl speed(speed_config.gain_a_per_rad_s, speed_config.max_current_a);
    const double speed_reference = speed_config.reference_rad_s.value_or(
        bus_voltage / (std::sqrt(3.0) * motor.back_emf_constant));

    const auto input = [&](const State& state) -> pmsm::DriveInput {
        return {{pi.id_reference_a, speed.output(speed_reference, state[2])},
                sim.load_torque_nm, bus_voltage};
    };
    const auto derivative = [&](const State& state) {
        return drive.evaluate(state, input(state)).derivative;
    };

    csv::CSVWriter<std::ostream, false> writer(output);
    const std::array<std::string, 18> columns{
        "time_s", "ia_a", "ib_a", "ic_a", "omega_rad_s", "theta_rad", "id_a", "iq_a",
        "id_ref_a", "iq_ref_a", "vd_v", "vq_v", "duty_a", "duty_b", "duty_c", "va_v", "vb_v", "vc_v",
    };
    writer << columns;

    State state = State::Zero();
    for (int step = 0; ; ++step) {
        if (step % sim.log_every_steps == 0 || step == sim.steps) {
            const auto command = input(state);
            const auto sample = drive.evaluate(state, command);
            const auto& currents = sample.motor.current.abc;
            const auto& duty = sample.duty.abc;
            const auto& voltage = sample.voltage.abc;
            const std::array<CsvNumber, columns.size()> row{
                step * sim.step_s, currents[0], currents[1], currents[2],
                sample.motor.omega, sample.motor.theta, sample.measured.d, sample.measured.q,
                command.reference.d, command.reference.q, sample.command.d, sample.command.q,
                duty[0], duty[1], duty[2], voltage[0], voltage[1], voltage[2],
            };
            writer << row;
            if (!output) {
                throw std::runtime_error("Could not write CSV row");
            }
        }
        if (step == sim.steps) {
            break;
        }
        state = pmsm::rk4_step(state, sim.step_s, derivative);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 3) {
        std::cerr << "Usage: pmsm_sim [output.csv [config.json]]\n";
        return 2;
    }

    const char* workspace = std::getenv("BUILD_WORKSPACE_DIRECTORY");
    const std::filesystem::path base = workspace ? workspace : ".";
    const auto output_path = base / (argc >= 2 ? argv[1] : "pmsm_results.csv");
    const auto config_path = base / (argc == 3 ? argv[2] : "pmsm_config.json");
    pmsm::Configuration config;
    try {
        config = pmsm::load_configuration(config_path);
    } catch (const std::exception& error) {
        std::cerr << "Invalid configuration " << config_path.string() << ": " << error.what() << '\n';
        return 1;
    }

    try {
        if (std::filesystem::exists(output_path) &&
            std::filesystem::equivalent(output_path, config_path)) {
            throw std::runtime_error("Output path refers to the configuration file");
        }
        std::ofstream output(output_path, std::ios::binary);
        if (!output) {
            throw std::runtime_error("Could not open CSV file");
        }
        simulate(output, config);
        output.close();
        if (!output) {
            throw std::runtime_error("Could not finish writing CSV");
        }
    } catch (const std::exception& error) {
        std::cerr << "Could not save " << output_path.string() << ": " << error.what() << '\n';
        return 1;
    }
    std::cout << "Saved " << output_path.string() << '\n';
}
