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

#include "blocks/continuous_pmsm.h"
#include "blocks/speed_control.h"
#include "configuration.h"
#include "drive.h"
#include "simulation_executor.h"

namespace {

using Executor = pmsm::SimulationExecutor;
using Duration = Executor::Duration;

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

class CsvLogger {
public:
    CsvLogger(std::ostream& output, const Executor& executor,
              const pmsm::CurrentControlledPmsm& drive)
        : output_(output), writer_(output), executor_(executor), drive_(drive) {
        writer_ << columns_;
    }

    void tick() {
        const auto& motor = drive_.motor_outputs();
        const auto& measured = drive_.feedback_outputs();
        const auto& command = drive_.pi_outputs();
        const auto& duty = drive_.pwm_outputs();
        const auto& voltage = drive_.inverter_outputs();
        const auto reference = drive_.current_reference();
        const std::array<CsvNumber, columns_.size()> row{
            std::chrono::duration<double>(executor_.now()).count(),
            motor.ia.value, motor.ib.value, motor.ic.value, motor.omega.value, motor.theta.value,
            measured.id.value, measured.iq.value, reference.d, reference.q,
            command.vd.value, command.vq.value, duty.duty_a.value, duty.duty_b.value,
            duty.duty_c.value, voltage.va.value, voltage.vb.value, voltage.vc.value,
        };
        writer_ << row;
        if (!output_) {
            throw std::runtime_error("Could not write CSV row");
        }
    }

private:
    inline static const std::array<std::string, 18> columns_{
        "time_s", "ia_a", "ib_a", "ic_a", "omega_rad_s", "theta_rad", "id_a", "iq_a",
        "id_ref_a", "iq_ref_a", "vd_v", "vq_v", "duty_a", "duty_b", "duty_c", "va_v", "vb_v", "vc_v",
    };
    std::ostream& output_;
    csv::CSVWriter<std::ostream, false> writer_;
    const Executor& executor_;
    const pmsm::CurrentControlledPmsm& drive_;
};

void simulate(const std::filesystem::path& path, const pmsm::Configuration& config) {
    const auto& motor = config.motor;
    const auto& pi = config.current_pi;
    const auto& sim = config.simulation;
    const auto& speed_config = config.speed_control;
    const double bus_voltage = config.inverter.bus_voltage_v;
    const auto max_step = sim.integration_step();
    const Duration log_period = sim.log_period ? *sim.log_period
                                               : max_step * sim.log_every_steps;
    const Duration current_period = pi.period;
    const auto end = max_step * sim.steps;

    Executor executor(max_step);
    pmsm::ContinuousPmsm continuous(pmsm::PmsmPlant(motor), sim.load_torque_nm);
    const pmsm::CurrentPi controller({
        pi.kp, pi.ki, pi.kaw, motor.inductance, motor.pole_pairs, motor.back_emf_constant,
    });
    const pmsm::SpeedControl speed(speed_config.gain_a_per_rad_s, speed_config.max_current_a);
    const double speed_reference = speed_config.reference_rad_s.value_or(
        bus_voltage / (std::sqrt(3.0) * motor.back_emf_constant));
    pmsm::CurrentControlledPmsm drive(
        continuous, controller, speed, bus_voltage, speed_reference, pi.id_reference_a,
        current_period, config.connections.value());

    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("Could not open CSV file");
    }
    {
        CsvLogger logger(output, executor, drive);
        enum Priority { speed_tick, feedback_tick, current_tick, pwm_tick, inverter_tick, logging_tick };
        executor.register_continuous(
            Executor::AdvanceCallback::bind<&pmsm::CurrentControlledPmsm::advance>(drive));
        executor.register_update(speed_config.period, speed_tick,
            Executor::UpdateCallback::bind<&pmsm::CurrentControlledPmsm::tick_speed>(drive));
        executor.register_update(current_period, feedback_tick,
            Executor::UpdateCallback::bind<&pmsm::CurrentControlledPmsm::tick_feedback>(drive));
        executor.register_update(current_period, current_tick,
            Executor::UpdateCallback::bind<&pmsm::CurrentControlledPmsm::tick_current>(drive));
        executor.register_update(config.inverter.pwm_period, pwm_tick,
            Executor::UpdateCallback::bind<&pmsm::CurrentControlledPmsm::tick_svpwm>(drive));
        executor.register_update(config.inverter.pwm_period, inverter_tick,
            Executor::UpdateCallback::bind<&pmsm::CurrentControlledPmsm::tick_inverter>(drive));
        executor.register_update(log_period, logging_tick,
            Executor::UpdateCallback::bind<&CsvLogger::tick>(logger));

        executor.run(end);
        if (end % log_period != Duration::zero()) {
            logger.tick();
        }
    }
    output.close();
    if (!output) {
        throw std::runtime_error("Could not finish writing CSV");
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
        if (!config.connections) {
            config.connections = pmsm::load_configuration(base / "pmsm_config.json").connections;
            if (!config.connections) {
                throw std::invalid_argument("No connections in the supplied or default configuration");
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "Invalid configuration " << config_path.string() << ": " << error.what() << '\n';
        return 1;
    }

    try {
        if (std::filesystem::exists(output_path) &&
            std::filesystem::equivalent(output_path, config_path)) {
            throw std::runtime_error("Output path refers to the configuration file");
        }
        simulate(output_path, config);
    } catch (const std::exception& error) {
        std::cerr << "Could not save " << output_path.string() << ": " << error.what() << '\n';
        return 1;
    }
    std::cout << "Saved " << output_path.string() << '\n';
}
