#ifndef STEPPER_GENERATOR_MOCK_HPP
#define STEPPER_GENERATOR_MOCK_HPP

#include <functional>

#include "Generic/Generator.hpp"

namespace Stepper {
namespace Mock {

class Generator : public Stepper::Generic::Generator {
public:
    using WaitHook = std::function<void(Stepper::State targetState)>;

    Generator() = delete;
    explicit Generator(Generic::Driver& driver)
        : Stepper::Generic::Generator(driver) {}

    void setWaitHook(WaitHook hook) {
        waitHook_ = std::move(hook);
    }

    Stepper::State waitForState(Stepper::State state, uint32_t /*timeout_ms*/ = UINT32_MAX) const override {
        constexpr int kMaxWaitIterations = 1000;
        for (int i = 0; i < kMaxWaitIterations; ++i) {
            if (getState().currentState == state) {
                return state;
            }
            if (waitHook_) {
                waitHook_(state);
            } else {
                break;
            }
        }
        return getState().currentState;
    }

protected:
    void updateStateEvent(Stepper::State /*state*/) const override {}

private:
    WaitHook waitHook_ {nullptr};
};

} // namespace Mock
} // namespace Stepper

#endif // STEPPER_GENERATOR_MOCK_HPP
