#ifndef STEPPER_ENDSTOP_HPP
#define STEPPER_ENDSTOP_HPP

#include <cstdint>
#include <cassert>
#include <functional>

#include "Helper.hpp"
#include "Pin.hpp"


namespace Stepper {
namespace Generic {

class Endstop {
public:
    enum class Edge : uint8_t {
        None = 0,
        Rising = 1,
        Falling = 2,
        Both = 3,
        Any = Both
    };

    using EndstopCallback = std::function<void(Edge edge, void* user_ctx)>;
    using SimpleCallback = std::function<void(Edge edge)>;
    using VoidCallback = std::function<void()>;

    Endstop() = delete;
    explicit Endstop(Pin& pin)
            : pin_(pin) {
        assert(pin_.getIoMode() == Pin::IoMode::Input || pin_.getIoMode() == Pin::IoMode::Input_Output);
    }

    virtual ~Endstop() = default;

    virtual void enableInterrupt() = 0;
    virtual void disableInterrupt() = 0;

    bool isTriggered() const {
        return pin_.isEnabled();
    };

    bool getLevel() const {
        return pin_.getLevel();
    };

    Pin& getPin() {
        return pin_;
    };

    const Pin& getPin() const {
        return pin_;
    };

    void setPullMode(Pin::PullMode pullMode) {
        pin_.setPullMode(pullMode);
    };

    Pin::PullMode getPullMode() const {
        return pin_.getPullMode();
    };

    void setDebounceUs(uint32_t debounce_us) {
        debounce_us_ = debounce_us;
    };

    uint32_t getDebounceUs() const {
        return debounce_us_;
    };

    // Register callback with Edge, callback, and user context
    void registerCallback(Edge edge, EndstopCallback callback, void* user_ctx = nullptr) {
        if (compare_enums(edge, Edge::Rising)) {
            callbackRising_ = callback;
            callbackRisingUserCtx_ = user_ctx;
        }
        if (compare_enums(edge, Edge::Falling)) {
            callbackFalling_ = callback;
            callbackFallingUserCtx_ = user_ctx;
        }
        enableInterrupt();
    };

    // Register callback for both rising and falling edges
    void registerCallback(EndstopCallback callback, void* user_ctx = nullptr) {
        registerCallback(Edge::Both, callback, user_ctx);
    };

    // Register callback on rising edge only
    void registerCallbackOnRising(EndstopCallback callback, void* user_ctx = nullptr) {
        registerCallback(Edge::Rising, callback, user_ctx);
    };

    // Register callback on falling edge only
    void registerCallbackOnFalling(EndstopCallback callback, void* user_ctx = nullptr) {
        registerCallback(Edge::Falling, callback, user_ctx);
    };

    // Overloads for SimpleCallback (void(Edge))
    void registerCallback(Edge edge, SimpleCallback callback) {
        registerCallback(edge, [callback](Edge e, void*) {
            if (callback) callback(e);
        }, nullptr);
    };

    void registerCallback(SimpleCallback callback) {
        registerCallback(Edge::Both, callback);
    };

    void registerCallbackOnRising(SimpleCallback callback) {
        registerCallback(Edge::Rising, callback);
    };

    void registerCallbackOnFalling(SimpleCallback callback) {
        registerCallback(Edge::Falling, callback);
    };

    // Overloads for VoidCallback (void())
    void registerCallback(Edge edge, VoidCallback callback) {
        registerCallback(edge, [callback](Edge, void*) {
            if (callback) callback();
        }, nullptr);
    };

    void registerCallback(VoidCallback callback) {
        registerCallback(Edge::Both, callback);
    };

    void registerCallbackOnRising(VoidCallback callback) {
        registerCallback(Edge::Rising, callback);
    };

    void registerCallbackOnFalling(VoidCallback callback) {
        registerCallback(Edge::Falling, callback);
    };

    // Overload for nullptr to unregister cleanly
    void registerCallback(std::nullptr_t) {
        unregisterCallback(Edge::Both);
    };

    // Unregister callback
    void unregisterCallback(Edge edge = Edge::Both) {
        if (compare_enums(edge, Edge::Rising)) {
            callbackRising_ = nullptr;
            callbackRisingUserCtx_ = nullptr;
        }
        if (compare_enums(edge, Edge::Falling)) {
            callbackFalling_ = nullptr;
            callbackFallingUserCtx_ = nullptr;
        }
        if (!hasCallback()) {
            disableInterrupt();
        }
    };

    bool hasCallback(Edge edge = Edge::Both) const {
        if (edge == Edge::Rising)
            return callbackRising_ != nullptr;

        if (edge == Edge::Falling)
            return callbackFalling_ != nullptr;

        return (callbackRising_ != nullptr && callbackFalling_ != nullptr);
    };

    // Trigger registered callbacks for a specific edge
    void triggerCallback(Edge edge) {
        if (callbackRising_ && compare_enums(edge, Edge::Rising)) {
            callbackRising_(Edge::Rising, callbackRisingUserCtx_);
        }
        if (callbackFalling_ && compare_enums(edge, Edge::Falling)) {
            callbackFalling_(Edge::Falling, callbackFallingUserCtx_);
        }
    };

protected:
    Pin& pin_;
    uint32_t debounce_us_ {0};

    void* callbackRisingUserCtx_ {nullptr};
    EndstopCallback callbackRising_ {nullptr};

    void* callbackFallingUserCtx_ {nullptr};
    EndstopCallback callbackFalling_ {nullptr};
};

} // namespace Generic
} // namespace Stepper

#endif // STEPPER_ENDSTOP_HPP