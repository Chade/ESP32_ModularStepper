#ifndef STEPPER_ENDSTOP_MOCK_HPP
#define STEPPER_ENDSTOP_MOCK_HPP

#include "Generic/Endstop.hpp"
#include "StepperPin_Mock.hpp"

namespace Stepper {
namespace Mock {

class Endstop : public Generic::Endstop {
public:
    Endstop() = delete;
    explicit Endstop(Generic::Pin& pin)
        : Generic::Endstop(pin) {}

    void enableInterrupt() override { enabled_ = true; }
    void disableInterrupt() override { enabled_ = false; }
    bool isInterruptEnabled() const { return enabled_; }

    void simulateEdge(Edge edge) {
        if (!enabled_) {
            return;
        }
        triggerCallback(edge);
    }

    void simulateTrigger(bool triggered) {
        if (triggered) {
            pin_.enable();
            simulateEdge(Edge::Rising);
        } else {
            pin_.disable();
            simulateEdge(Edge::Falling);
        }
    }

    bool enabled_ {true};
};

} // namespace Mock
} // namespace Stepper

#endif // STEPPER_ENDSTOP_MOCK_HPP
