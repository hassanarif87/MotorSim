#pragma once

namespace pmsm {

/// @brief Represents an input port for a block in the simulation.
/// @tparam T The type of value stored in the port.
template <typename T>
class InputPort {
  public:
    using Value = T;

    const T& read() const { return *source_; }
    bool connected() const { return source_ != nullptr; }
    void bind(const T& source) { source_ = &source; }

  private:
    /// @brief Pointer to the source value for the input port. If nullptr, the port is not
    /// connected.
    const T* source_ = nullptr;
};

namespace drive_blocks {
/// @brief Carries a scalar with a quantity tag for type-checked block connections.
/// @tparam Quantity The tag that identifies the signal's quantity.
template <typename Quantity>
struct Signal {
    Signal() = default;
    Signal(double initial) : value(initial) {}

    operator double() const { return value; }

    double value = 0.0;
};

using Voltage = Signal<struct VoltageTag>;
using Current = Signal<struct CurrentTag>;
using Torque = Signal<struct TorqueTag>;
using Speed = Signal<struct SpeedTag>;
using Position = Signal<struct PositionTag>;
using Duty = Signal<struct DutyTag>;
using Gate = Signal<struct GateTag>;

/// @brief Port list for a source block with no inputs.
struct NoInputs {};

} // namespace drive_blocks
} // namespace pmsm
