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
            add_port(inputs_, block + "." + member.name, Input{
                typeid(Value), &port,
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
            add_port(outputs_, block + "." + member.name, Output{typeid(port), &port});
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
            input.bind(input.address, output.address);
        }

        for (const auto& [name, input] : inputs_) {
            if (!input.connected(input.address)) {
                throw std::invalid_argument("Missing source for input port: " + name);
            }
        }
    }

private:
    struct Input {
        std::type_index type;
        void* address;
        void (*bind)(void*, const void*);
        bool (*connected)(const void*);
    };

    struct Output {
        std::type_index type;
        const void* address;
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
