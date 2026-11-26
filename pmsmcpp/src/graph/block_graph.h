#pragma once

#include <meta>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeindex>
#include <unordered_map>

#include "src/graph/block_ports.h"
#include "src/graph/connections.h"

namespace pmsm {

/// @brief Represents a reader for an output port, allowing retrieval of its value as a double.
struct OutputReader {
    const void* address;
    double (*read)(const void*);

    double operator()() const { return read(address); }
};

// Startup-only registry. Each input retains a typed pointer to its source.
class BlockGraph {
  public:
    template <typename Inputs, typename Outputs>
    void add(const std::string& block, Inputs& inputs, const Outputs& outputs) {
        visit_ports(inputs, [&](std::string_view name, auto& port) {
            using Port = std::remove_reference_t<decltype(port)>;
            using Value = typename Port::Value;
            OutputReader reader{&port, nullptr};
            if constexpr (std::is_convertible_v<Value, double>) {
                reader.read = [](const void* address) -> double {
                    return static_cast<const Port*>(address)->read();
                };
            }
            add_port(
                inputs_, (block + ".").append(name),
                InputPortEntry{
                    typeid(Value),
                    &port,
                    reader,
                    [](void* destination, const void* source) {
                        static_cast<Port*>(destination)->bind(*static_cast<const Value*>(source));
                    },
                    [](const void* destination) {
                        return static_cast<const Port*>(destination)->connected();
                    },
                });
        });

        visit_ports(outputs, [&](std::string_view name, const auto& port) {
            using Value = std::decay_t<decltype(port)>;
            OutputReader reader{&port, nullptr};
            if constexpr (std::is_convertible_v<Value, double>) {
                reader.read = [](const void* address) -> double {
                    return *static_cast<const Value*>(address);
                };
            }
            add_port(outputs_, (block + ".").append(name), OutputPortEntry{typeid(Value), reader});
        });
    }

    void connect(const Connections& connections) const {
        for (const auto& [destination, source] : connections) {
            const auto in = inputs_.find(destination);
            if (in == inputs_.end()) {
                throw std::invalid_argument("Unknown input port: " + destination);
            }
            const auto out = outputs_.find(source);
            if (out == outputs_.end()) {
                throw std::invalid_argument("Unknown output port: " + source);
            }
            const auto& input = in->second;
            const auto& output = out->second;
            if (input.type != output.type) {
                throw std::invalid_argument("Incompatible port types: " + source + " -> " +
                                            destination);
            }
            if (input.connected(input.address)) {
                throw std::invalid_argument("Multiple sources for input port: " + destination);
            }
            input.bind(input.address, output.reader.address);
        }

        for (const auto& [name, input] : inputs_) {
            if (!input.connected(input.address)) {
                throw std::invalid_argument("Missing source for input port: " + name);
            }
        }
    }

    OutputReader numeric_port(const std::string& name) const {
        const auto output = outputs_.find(name);
        if (output != outputs_.end() && output->second.reader.read) {
            return output->second.reader;
        }
        const auto input = inputs_.find(name);
        if (input != inputs_.end() && input->second.reader.read) {
            return input->second.reader;
        }
        throw std::invalid_argument("Unknown or nonnumeric port for CSV: " + name);
    }

  private:
    template <typename Ports, typename Visit>
    static void visit_ports(Ports& ports, [[maybe_unused]] Visit visit) {
        using PortList = std::remove_cv_t<Ports>;
        static constexpr auto members = std::define_static_array(
            std::meta::nonstatic_data_members_of(^^PortList, std::meta::access_context::current()));
        template for (constexpr auto member : members) {
            visit(std::meta::identifier_of(member), ports.[:member:]);
        }
    }

    /// @brief Represents a single input port in the block graph, including its type, address,
    /// reader, and functions for binding and checking connection status.
    struct InputPortEntry {
        std::type_index type;
        void* address;
        OutputReader reader;
        void (*bind)(void*, const void*);
        bool (*connected)(const void*);
    };

    struct OutputPortEntry {
        std::type_index type;
        OutputReader reader;
    };

    template <typename Port>
    static void add_port(std::unordered_map<std::string, Port>& registry, const std::string& name,
                         const Port& port) {
        if (!registry.emplace(name, port).second) {
            throw std::invalid_argument("Duplicate port: " + name);
        }
    }

    std::unordered_map<std::string, InputPortEntry> inputs_;
    std::unordered_map<std::string, OutputPortEntry> outputs_;
};

} // namespace pmsm
