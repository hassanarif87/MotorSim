#pragma once

#include <utility>    // std forward
#include <functional> // std invoke
namespace pmsm {

/// @brief A delegate class for binding member functions to callbacks.
/// @tparam Signature The signature of the function to be bound.
template <typename Signature>
class Delegate;

/// @brief Delegate class for binding member functions to callbacks.
/// @tparam ...Args The argument types of the bound member function.
template <typename... Args>
class Delegate<void(Args...)> {
  public:
    /// @brief Binds a member function to the delegate.
    /// @tparam Object The type of the object whose member function is being bound.
    /// @tparam Method The member function pointer to bind.
    /// @param object The object whose member function is being bound.
    /// @return Bound delegate that can be invoked with the specified arguments.
    template <auto Callable, typename Object>
    static Delegate bind(Object& object) {
        return {&object, [](void* context, Args... args) {
                    // Restore the object from the context and invoke the member function with the
                    // provided arguments.
                    auto& bound = *static_cast<Object*>(context);
                    // Use std::invoke to call the member function on the object with the provided
                    // arguments.
                    std::invoke(Callable, bound, std::forward<Args>(args)...);
                }};
    }

    /// @brief Checks if the delegate is valid (i.e., has a bound function).
    explicit operator bool() const { return invoke_ != nullptr; }
    /// @brief Invokes the bound member function with the specified arguments.
    /// @param ...args The arguments to pass to the bound member function.
    void operator()(Args... args) const { invoke_(context_, std::forward<Args>(args)...); }

    Delegate() = default;

  private:
    /// @brief Constructs a delegate with the given context and invocation function.
    /// @tparam ...Args The argument types of the bound member function.
    /// @param context The context (object) on which the member function will be invoked.
    /// @param invoke The invocation function.
    Delegate(void* context, void (*invoke)(void*, Args...)) : context_(context), invoke_(invoke) {}

    /// @brief The context (object) on which the member function will be invoked.
    void* context_ = nullptr;
    /// @brief The invocation function that calls the bound member function with the provided
    /// arguments.
    void (*invoke_)(void*, Args...) = nullptr;
};
} // namespace pmsm