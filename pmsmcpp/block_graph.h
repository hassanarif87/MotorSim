#pragma once

#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeindex>
#include <unordered_map>

#include <boost/describe.hpp>
#include <boost/mp11.hpp>
#include "connections.h"

namespace pmsm {

template <typename T>
class InputPort {
public:
    using Value = T;

    const T& read() const { return *source_; }
    bool connected() const { return source_ != nullptr; }
    void bind(const T& source) { source_ = &source; }

private:
    const T* source_ = nullptr;
};

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
        using boost::describe::describe_members;
        using boost::describe::mod_public;

        boost::mp11::mp_for_each<describe_members<Inputs, mod_public>>([&](auto member) {
            auto& port = inputs.*member.pointer;
            using Port = std::remove_reference_t<decltype(port)>;
            using Value = typename Port::Value;
            OutputReader reader{&port, nullptr};
            if constexpr (std::is_convertible_v<Value, double>) {
                reader.read = [](const void* address) -> double {
                    return static_cast<const Port*>(address)->read();
                };
            }
            add_port(inputs_, block + "." + member.name, Input{
                typeid(Value), &port, reader,
                [](void* destination, const void* source) {
                    static_cast<Port*>(destination)->bind(*static_cast<const Value*>(source));
                },
                [](const void* destination) {
                    return static_cast<const Port*>(destination)->connected();
                },
            });
        });

        boost::mp11::mp_for_each<describe_members<Outputs, mod_public>>([&](auto member) {
            const auto& port = outputs.*member.pointer;
            using Value = std::decay_t<decltype(port)>;
            OutputReader reader{&port, nullptr};
            if constexpr (std::is_convertible_v<Value, double>) {
                reader.read = [](const void* address) -> double {
                    return *static_cast<const Value*>(address);
                };
            }
            add_port(outputs_, block + "." + member.name, Output{typeid(Value), reader});
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
    struct Input {
        std::type_index type;
        void* address;
        OutputReader reader;
        void (*bind)(void*, const void*);
        bool (*connected)(const void*);
    };

    struct Output {
        std::type_index type;
        OutputReader reader;
    };

    template <typename Port>
    static void add_port(std::unordered_map<std::string, Port>& registry,
                         const std::string& name, const Port& port) {
        if (!registry.emplace(name, port).second) {
            throw std::invalid_argument("Duplicate port: " + name);
        }
    }

    std::unordered_map<std::string, Input> inputs_;
    std::unordered_map<std::string, Output> outputs_;
};

}  // namespace pmsm
