#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "configuration.h"
#include "csv_logger.h"
#include "drive/drive.h"
#include "simulation_executor.h"

namespace {

using Executor = pmsm::SimulationExecutor;
using Duration = Executor::Duration;

void simulate(const std::filesystem::path& path, const pmsm::Configuration& config) {
    const auto& sim = config.simulation;
    const auto max_step = sim.integration_step();
    const Duration log_period = sim.log_period ? *sim.log_period : max_step * sim.log_every_steps;
    const auto end = max_step * sim.steps;
    pmsm::Drive drive(config);
    auto& executor = drive.executor();

    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("Could not open CSV file");
    }
    {
        pmsm::CsvLogger logger(output, executor, drive.columns());
        executor.register_block(logger, log_period, std::numeric_limits<int>::max());

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
        if (!config.connections || !config.log_columns) {
            const auto defaults = pmsm::load_configuration(base / "pmsm_config.json");
            if (!config.connections) {
                config.connections = defaults.connections;
            }
            if (!config.log_columns) {
                config.log_columns = defaults.log_columns;
            }
            if (!config.connections || !config.log_columns) {
                throw std::invalid_argument("The supplied or default configuration must define connections and log_columns");
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
