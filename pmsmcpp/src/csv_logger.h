#pragma once

#include <ostream>
#include <string>
#include <vector>

#include <csv.hpp>

#include "src/drive/drive.h"
#include "src/simulation_executor.h"

namespace pmsm {

class CsvLogger {
public:
    CsvLogger(std::ostream& output, const SimulationExecutor& executor,
              const std::vector<Drive::Column>& columns);

    void tick();

private:
    // Give the CSV library a round-trippable field representation.
    struct CsvNumber {
        CsvNumber(double field) : value(field) {}

        operator std::string() const;

        double value;
    };

    std::ostream& output_;
    csv::CSVWriter<std::ostream, false> writer_;
    const SimulationExecutor& executor_;
    const std::vector<Drive::Column>& columns_;
    std::vector<CsvNumber> row_;
};

}  // namespace pmsm
