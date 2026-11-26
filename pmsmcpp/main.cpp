#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "src/configuration.h"
#include "src/csv_logger.h"
#include "src/drive/drive.h"
#include "src/simulation_executor.h"

namespace {

using Executor = pmsm::SimulationExecutor;
using Duration = Executor::Duration;

/// @brief Simulates the PMSM model and saves the results to a CSV file.
/// @param path The path to the output CSV file.
/// @param config The simulation configuration.
void simulate(const std::filesystem::path& path, const pmsm::Configuration& config) {
    const auto& sim = config.simulation;
    const Duration log_period = sim.logging_period;
    const auto end = sim.duration;
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

} // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: pmsm_sim config.json [output.csv]\n";
        return 2;
    }

    const char* workspace = std::getenv("BUILD_WORKSPACE_DIRECTORY");
    const std::filesystem::path base = workspace ? workspace : ".";
    const auto config_path = base / argv[1];
    const auto output_path = base / (argc == 3 ? argv[2] : "pmsm_results.csv");
    std::optional<pmsm::Configuration> config;
    try {
        config.emplace(pmsm::load_configuration(config_path));
    } catch (const std::exception& error) {
        std::cerr << "Invalid configuration " << config_path.string() << ": " << error.what()
                  << '\n';
        return 1;
    }

    try {
        if (std::filesystem::exists(output_path) &&
            std::filesystem::equivalent(output_path, config_path)) {
            throw std::runtime_error("Output path refers to the configuration file");
        }
        simulate(output_path, *config);
    } catch (const std::exception& error) {
        std::cerr << "Could not save " << output_path.string() << ": " << error.what() << '\n';
        return 1;
    }
    std::cout << "Saved " << output_path.string() << '\n';
}
