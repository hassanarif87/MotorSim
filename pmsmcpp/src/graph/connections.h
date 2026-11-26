#pragma once

#include <array>
#include <string>
#include <vector>

namespace pmsm {

/// @brief Represents a collection of connections between blocks in the simulation. E.g. a
/// connection from block A to block B is represented as a pair of strings {"A", "B"}.
using Connections = std::vector<std::array<std::string, 2>>;

} // namespace pmsm
