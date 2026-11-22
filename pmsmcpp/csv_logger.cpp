#include "csv_logger.h"

#include <charconv>
#include <chrono>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace pmsm {

CsvLogger::CsvNumber::operator std::string() const {
    char buffer[64];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value,
        std::chars_format::general, std::numeric_limits<double>::max_digits10);
    if (result.ec != std::errc{}) {
        throw std::runtime_error("Could not format CSV number");
    }
    return {buffer, result.ptr};
}

CsvLogger::CsvLogger(std::ostream& output, const SimulationExecutor& executor,
                     const std::vector<Drive::Column>& columns)
    : output_(output), writer_(output), executor_(executor), columns_(columns),
      row_(columns.size() + 1, CsvNumber(0.0)) {
    std::vector<std::string> header{"time_s"};
    for (const auto& column : columns_) {
        header.push_back(column.header);
    }
    writer_ << header;
}

void CsvLogger::tick() {
    row_[0] = std::chrono::duration<double>(executor_.now()).count();
    for (std::size_t i = 0; i < columns_.size(); ++i) {
        row_[i + 1] = columns_[i].source();
    }
    writer_ << row_;
    if (!output_) {
        throw std::runtime_error("Could not write CSV row");
    }
}

}  // namespace pmsm
