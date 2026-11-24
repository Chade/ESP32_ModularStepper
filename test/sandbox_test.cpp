#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include <FixedPoints.h>
#include <FixedPointsCommon.h>

#include "Generic/Axis.hpp"
#include "Generic/Axis_Linear.hpp"
#include "Generic/Axis_Rotary.hpp"
#include "Generic/Helper.hpp"
#include "Generic/Homing.hpp"
#include "Generic/Motor.hpp"
#include "Generic/StopController.hpp"
#include "StepperDriver_Mock.hpp"
#include "StepperEndstop_Mock.hpp"
#include "StepperGenerator_Mock.hpp"

using UQ20x12 = UFixed<20, 12>;

namespace {
int g_passedTests = 0;
int g_failedTests = 0;
} // namespace

void printTestResult(const std::string& testName, bool success) {
    if (success) {
        g_passedTests++;
        std::cout << "[PASS] " << testName << std::endl;
    } else {
        g_failedTests++;
        std::cout << "[FAIL] " << testName << std::endl;
    }
}

void testEnumHelpers() {
    std::cout << "\n=== Testing StepperHelper Enums ===" << std::endl;

    auto dirVal = Stepper::cast_enum_to_base(Stepper::Direction::Counterclockwise);
    printTestResult("Direction cast_enum_to_base (Counterclockwise == 1)", dirVal == 1);

    auto stateVal = Stepper::cast_enum_to_base(Stepper::State::Accelerating);
    printTestResult("State cast_enum_to_base (Accelerating == 6)", stateVal == 6);

    auto compared = Stepper::compare_enums(Stepper::State::Accelerating, Stepper::State::Running);
    printTestResult("State bitwise compare_enums", compared == (6 & 2));
}

void testFixedPointSquareroot() {
    std::cout << "\n=== Testing Squareroot Calculation ===" << std::endl;

    UQ20x12 v1 = 2.0;
    UQ20x12 v2 = 16.0;
    UQ20x12 v3 = 100.0;
    UQ20x12 v4 = 0.25;

    UQ20x12 result = Stepper::sqrt(v1);
    printTestResult("FixedPoint sqrt(2.0)", (result > 1.41 && result < 1.42));
    printTestResult("FixedPoint sqrt(16.0)", Stepper::sqrt(v2) == 4.0);
    printTestResult("FixedPoint sqrt(100.0)", Stepper::sqrt(v3) == 10.0);
    printTestResult("FixedPoint sqrt(0.25)", Stepper::sqrt(v4) == 0.5);

    printTestResult("isqrt64(0) == 0", Stepper::isqrt64(0) == 0);
    printTestResult("isqrt64(1) == 1", Stepper::isqrt64(1) == 1);
    printTestResult("isqrt64(2) == 1", Stepper::isqrt64(2) == 1);
    printTestResult("isqrt64(3) == 1", Stepper::isqrt64(3) == 1);
    printTestResult("isqrt64(4) == 2", Stepper::isqrt64(4) == 2);
    printTestResult("isqrt64(0xFFFFFFFF) == 65535", Stepper::isqrt64(0xFFFFFFFFULL) == 65535);
    printTestResult("isqrt64(0x100000000) == 65536", Stepper::isqrt64(0x100000000ULL) == 65536);
    printTestResult("isqrt64(UINT64_MAX) == 4294967295", Stepper::isqrt64(UINT64_MAX) == 4294967295ULL);
}

void testStepperGeneratorKinematics() {
    std::cout << "\n=== Testing Stepper Generator Kinematics ===" << std::endl;

    auto runToDone = [](Stepper::Mock::Generator& gen, Stepper::Mock::Driver& drv, const Stepper::Generic::Generator::GeneratorTask& task) {
        if (!gen.run(task)) {
            return false;
        }
        return drv.stepLoop();
    };

    // 1. Trapezoid step mode (1000 steps)
    {
        Stepper::Mock::Pin pin(0);
        Stepper::Mock::Driver driver(pin, pin, pin);
        Stepper::Mock::Generator generator(driver);
        generator.init();
        driver.enable();

        Stepper::Generic::Generator::GeneratorTask task;
        task.steps = 1000;
        task.velocity = 1000.0f;
        task.acceleration = 1000.0f;
        task.deceleration = 1000.0f;
        task.direction = Stepper::Direction::Counterclockwise;

        bool ok = runToDone(generator, driver, task);
        auto st = generator.getState();
        printTestResult("Trapezoid 1000-step move finishes Stopped", ok && st.currentState == Stepper::State::Stopped);
        printTestResult("Trapezoid 1000-step move reaches exact position", st.stepsPosition == 1000 && st.stepsDone == 1000);
    }

    // 2. Zero acceleration
    {
        Stepper::Mock::Pin pin(0);
        Stepper::Mock::Driver driver(pin, pin, pin);
        Stepper::Mock::Generator generator(driver);
        generator.init();
        driver.enable();

        Stepper::Generic::Generator::GeneratorTask task;
        task.steps = 20;
        task.velocity = 10.0f;
        task.acceleration = 0.0f;
        task.deceleration = 10.0f;
        task.direction = Stepper::Direction::Clockwise;

        bool ok = runToDone(generator, driver, task);
        auto st = generator.getState();
        printTestResult("Zero-acc 20-step move completes exactly", ok && st.stepsDone == 20 && st.stepsPosition == -20);
    }

    // 3. Zero deceleration
    {
        Stepper::Mock::Pin pin(0);
        Stepper::Mock::Driver driver(pin, pin, pin);
        Stepper::Mock::Generator generator(driver);
        generator.init();
        driver.enable();

        Stepper::Generic::Generator::GeneratorTask task;
        task.steps = 20;
        task.velocity = 10.0f;
        task.acceleration = 10.0f;
        task.deceleration = 0.0f;
        task.direction = Stepper::Direction::Clockwise;

        bool ok = runToDone(generator, driver, task);
        auto st = generator.getState();
        printTestResult("Zero-dec 20-step move completes exactly", ok && st.stepsDone == 20 && st.stepsPosition == -20);
    }

    // 4. Zero acceleration and deceleration
    {
        Stepper::Mock::Pin pin(0);
        Stepper::Mock::Driver driver(pin, pin, pin);
        Stepper::Mock::Generator generator(driver);
        generator.init();
        driver.enable();

        Stepper::Generic::Generator::GeneratorTask task;
        task.steps = 20;
        task.velocity = 10.0f;
        task.acceleration = 0.0f;
        task.deceleration = 0.0f;
        task.direction = Stepper::Direction::Clockwise;

        bool ok = runToDone(generator, driver, task);
        auto st = generator.getState();
        printTestResult("Zero-acc/dec 20-step move completes exactly", ok && st.stepsDone == 20 && st.stepsPosition == -20);
    }

    // 5. Batch step acceleration kinematics (stepsAcc=500, stepsConst=1000, stepsDec=500)
    {
        Stepper::Mock::Pin pin(0);
        Stepper::Mock::Driver driver(pin, pin, pin);
        Stepper::Mock::Generator generator(driver);
        generator.init();
        driver.enable();

        Stepper::Generic::Generator::GeneratorTask task;
        task.steps = 2000;
        task.velocity = 1000.0f;
        task.acceleration = 1000.0f;
        task.deceleration = 1000.0f;
        task.direction = Stepper::Direction::Clockwise;

        generator.run(task);
        generator.advanceStateAfterStep(250);
        float v1 = generator.getVelocity();
        generator.advanceStateAfterStep(250);
        float v2 = generator.getVelocity();
        generator.advanceStateAfterStep(1500);
        float v3 = generator.getVelocity();

        // After 250 steps: sqrt(v0^2 + 2*1000*250) ~ 708.5 steps/s; at 500 steps: 1000 steps/s; at 2000 steps: 0
        printTestResult("Batch Step Acceleration (250 -> ~708.5, 500 -> 1000, 2000 -> 0)",
                        v1 > 700.0f && v1 < 715.0f && v2 > 999.0f && v2 < 1001.0f && v3 < 1.0f);
    }
}

void testStepperEndstop() {
    std::cout << "\n=== Testing Stepper Endstop ===" << std::endl;

    Stepper::Mock::Pin pin(15);
    Stepper::Mock::Endstop endstop(pin);

    printTestResult("Endstop pin number", endstop.getPin().getPin() == 15);

    int risingCount = 0;
    int fallingCount = 0;
    endstop.registerCallback([&risingCount, &fallingCount](Stepper::Generic::Endstop::Edge edge, void*) {
        if (edge == Stepper::Generic::Endstop::Edge::Rising) {
            risingCount++;
        } else if (edge == Stepper::Generic::Endstop::Edge::Falling) {
            fallingCount++;
        }
    });

    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Falling);
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);

    printTestResult("Both-edge callback rising count == 2", risingCount == 2);
    printTestResult("Both-edge callback falling count == 1", fallingCount == 1);

    int separateRisingCount = 0;
    int separateFallingCount = 0;
    void* ctxRising = reinterpret_cast<void*>(0x1234);
    void* ctxFalling = reinterpret_cast<void*>(0x5678);
    void* receivedCtxRising = nullptr;
    void* receivedCtxFalling = nullptr;

    endstop.registerCallbackOnRising([&separateRisingCount, &receivedCtxRising](Stepper::Generic::Endstop::Edge, void* ctx) {
        separateRisingCount++;
        receivedCtxRising = ctx;
    }, ctxRising);

    endstop.registerCallbackOnFalling([&separateFallingCount, &receivedCtxFalling](Stepper::Generic::Endstop::Edge, void* ctx) {
        separateFallingCount++;
        receivedCtxFalling = ctx;
    }, ctxFalling);

    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    printTestResult("Separate callback on rising triggers", separateRisingCount == 1 && separateFallingCount == 0);
    printTestResult("Separate callback receives rising ctx", receivedCtxRising == ctxRising);

    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Falling);
    printTestResult("Separate callback on falling triggers", separateRisingCount == 1 && separateFallingCount == 1);
    printTestResult("Separate callback receives falling ctx", receivedCtxFalling == ctxFalling);

    int simpleCallbackCount = 0;
    endstop.registerCallback([&simpleCallbackCount](Stepper::Generic::Endstop::Edge) {
        simpleCallbackCount++;
    });
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Falling);
    printTestResult("SimpleCallback called for both edges", simpleCallbackCount == 2);

    int voidRisingCount = 0;
    endstop.registerCallbackOnRising([&voidRisingCount]() {
        voidRisingCount++;
    });
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Falling);
    printTestResult("VoidCallback called for rising edge only", voidRisingCount == 1);

    endstop.disableInterrupt();
    int countWhileDisabled = voidRisingCount;
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    printTestResult("Disabled endstop suppresses callback", voidRisingCount == countWhileDisabled);
    endstop.enableInterrupt();
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    printTestResult("Re-enabled endstop triggers callback", voidRisingCount == countWhileDisabled + 1);

    endstop.unregisterCallback();
    int countAfterUnregister = voidRisingCount;
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Falling);
    printTestResult("Unregistered callback receives no events", voidRisingCount == countAfterUnregister);

    endstop.setPullMode(Stepper::Generic::Pin::PullMode::PullDown);
    printTestResult("PullMode set/get", endstop.getPullMode() == Stepper::Generic::Pin::PullMode::PullDown);

    endstop.setDebounceUs(1000);
    printTestResult("Debounce set/get", endstop.getDebounceUs() == 1000);
}

void testStepCountExactness() {
    std::cout << "\n=== Testing Step Count Exactness (90 deg @ 200 steps/rev x 16 microsteps) ===" << std::endl;

    Stepper::Mock::Pin pinEnable(1);
    Stepper::Mock::Pin pinStep(2);
    Stepper::Mock::Pin pinDir(3);
    Stepper::Mock::Driver driver(pinEnable, pinStep, pinDir, 16);
    Stepper::Mock::Generator generator(driver);
    generator.init();
    driver.enable();
    Stepper::Generic::Motor motor(generator, 200, 1.0f);
    Stepper::Generic::RotaryAxis axis(motor, 360.0f, true);

    axis.setPosition(0.0f);
    axis.run(90.0f, 2.0f, 20.0f, 20.0f);

    uint32_t executedSteps = 0;
    while (axis.getState() != Stepper::State::Stopped && executedSteps < 1000) {
        generator.advanceStateAfterStep(1);
        executedSteps++;
    }

    printTestResult("90 deg slow move executes exactly 800 microsteps", executedSteps == 800);
    printTestResult("90 deg slow move final angle == 90.0 deg", std::abs(axis.getAngleDeg() - 90.0f) < 0.001f);

    axis.setPosition(0.0f);
    axis.run(90.0f, 180.0f, 100.0f, 100.0f);
    executedSteps = 0;
    while (axis.getState() != Stepper::State::Stopped && executedSteps < 1000) {
        generator.advanceStateAfterStep(1);
        executedSteps++;
    }
    printTestResult("90 deg trapezoid move executes exactly 800 microsteps", executedSteps == 800);
    printTestResult("90 deg trapezoid move final angle == 90.0 deg", std::abs(axis.getAngleDeg() - 90.0f) < 0.001f);
}

void testRotaryAxisHoming() {
    std::cout << "\n=== Testing Rotary Axis and Homing ===" << std::endl;

    Stepper::Mock::Pin pinEnable(1);
    Stepper::Mock::Pin pinStep(2);
    Stepper::Mock::Pin pinDir(3);
    Stepper::Mock::Driver driver(pinEnable, pinStep, pinDir);
    Stepper::Mock::Generator generator(driver);
    generator.init();
    driver.enable();
    Stepper::Generic::Motor motor(generator, 200, 1.0f);
    Stepper::Generic::RotaryAxis axis(motor, 360.0f, false);

    axis.setLimits(-180.0f, 180.0f);
    printTestResult("RotaryAxis min angle", axis.getMinAngleDeg() == -180.0f);
    printTestResult("RotaryAxis max angle", axis.getMaxAngleDeg() == 180.0f);
    printTestResult("RotaryAxis initially not homed", !axis.isHomed());
    printTestResult("RotaryAxis initially not homing", !axis.isHoming());

    Stepper::Mock::Pin pinMin(10);
    Stepper::Mock::Pin pinMax(11);
    Stepper::Mock::Endstop endstopMin(pinMin);
    Stepper::Mock::Endstop endstopMax(pinMax);

    printTestResult("No min endstop initially", !axis.hasEndstopMin());
    printTestResult("No max endstop initially", !axis.hasEndstopMax());

    Stepper::Generic::Homing homing;
    printTestResult("Homing fails when endstop is not attached", !homing.homeMin(axis));
    printTestResult("Homing state is Failed", homing.getState() == Stepper::Generic::Homing::State::Failed);

    axis.setEndstopMin(&endstopMin);
    axis.setEndstopMax(&endstopMax);
    printTestResult("Has min endstop after set", axis.hasEndstopMin());
    printTestResult("Has max endstop after set", axis.hasEndstopMax());
    printTestResult("getEndstopMin matches", axis.getEndstopMin() == &endstopMin);
    printTestResult("getEndstopMax matches", axis.getEndstopMax() == &endstopMax);

    axis.setPosition(90.0f);
    printTestResult("setPosition / getPosition", axis.getPosition() == 90.0f);
    printTestResult("setPosition / getAngleDeg", axis.getAngleDeg() == 90.0f);

    Stepper::Generic::Homing::Config cfg;
    cfg.strategy = Stepper::Generic::Homing::Strategy::Min;
    cfg.fastVelocity = 100.0f;
    cfg.slowVelocity = 20.0f;
    cfg.acceleration = 200.0f;
    cfg.deceleration = 200.0f;
    cfg.retractDistance = 15.0f;
    cfg.backoffDistance = 5.0f;
    cfg.maxTravelDistance = 360.0f;
    cfg.homePosition = -180.0f;

    bool sawFinalBackoff = false;
    int waitTicks = 0;
    generator.setWaitHook([&](Stepper::State) {
        waitTicks++;
        auto hState = homing.getState();
        if (hState == Stepper::Generic::Homing::State::BackingOff) {
            endstopMin.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::FastApproach) {
            if (waitTicks >= 3) {
                endstopMin.simulateTrigger(true);
            }
        } else if (hState == Stepper::Generic::Homing::State::Retracting) {
            endstopMin.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::SlowApproach) {
            if (!endstopMin.isTriggered()) {
                endstopMin.simulateTrigger(true);
            } else {
                // Final backoff runs after SlowApproach sets homePosition while state_ remains SlowApproach
                sawFinalBackoff = true;
                uint32_t totalSteps = static_cast<uint32_t>(generator.getState().stepsTotal);
                if (totalSteps > 0) {
                    generator.advanceStateAfterStep(totalSteps);
                }
                endstopMin.simulateTrigger(false);
                axis.stop();
            }
        }
    });

    bool homeSuccess = homing.home(axis, cfg);
    printTestResult("Two-stage homing Min completes successfully", homeSuccess);
    printTestResult("Final back-off phase executed", sawFinalBackoff);
    printTestResult("Endstop is released after final back-off", !endstopMin.isTriggered());
    printTestResult("Axis is marked homed", axis.isHomed());
    printTestResult("Axis is not homing after completion", !axis.isHoming());
    printTestResult("Homing state is Done", homing.getState() == Stepper::Generic::Homing::State::Done);
    printTestResult("Axis position reflects backed-off angle (> -180 deg)", axis.getPosition() > -180.0f);
    printTestResult("Axis angle matches position", axis.getAngleDeg() == axis.getPosition());
    printTestResult("Generator stepsDone reset after homing", generator.getStepsDone() == 0);
    printTestResult("Generator stepsTotal reset after homing", generator.getState().stepsTotal == 0);
    printTestResult("Driver steps reset after homing", driver.getSteps() == 0);
    printTestResult("Axis state is Stopped after homing", axis.getState() == Stepper::State::Stopped);

    // Initial backoff when endstop is already triggered
    endstopMin.simulateTrigger(true);
    waitTicks = 0;
    generator.setWaitHook([&](Stepper::State) {
        waitTicks++;
        auto hState = homing.getState();
        if (hState == Stepper::Generic::Homing::State::BackingOff) {
            endstopMin.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::FastApproach) {
            endstopMin.simulateTrigger(true);
        } else if (hState == Stepper::Generic::Homing::State::Retracting) {
            endstopMin.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::SlowApproach) {
            if (!endstopMin.isTriggered()) {
                endstopMin.simulateTrigger(true);
            } else {
                endstopMin.simulateTrigger(false);
                axis.stop();
            }
        }
    });

    bool homeFromTriggered = homing.home(axis, cfg);
    printTestResult("Homing with initially triggered endstop succeeds", homeFromTriggered);
    printTestResult("Axis homed after initial backoff", axis.isHomed());

    // Max travel distance abort handling
    endstopMin.simulateTrigger(false);
    generator.setWaitHook([&](Stepper::State) {
        axis.stop();
    });
    bool homeMaxTravelAbort = homing.home(axis, cfg);
    printTestResult("Homing aborts if maxTravelDistance exceeded without trigger", !homeMaxTravelAbort);
    printTestResult("Homing state is Failed after maxTravelDistance abort", homing.getState() == Stepper::Generic::Homing::State::Failed);

    // Max Endstop Homing
    cfg.strategy = Stepper::Generic::Homing::Strategy::Max;
    cfg.homePosition = 180.0f;
    cfg.maxTravelDistance = 360.0f;
    generator.setWaitHook([&](Stepper::State) {
        auto hState = homing.getState();
        if (hState == Stepper::Generic::Homing::State::BackingOff) {
            endstopMax.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::FastApproach) {
            endstopMax.simulateTrigger(true);
        } else if (hState == Stepper::Generic::Homing::State::Retracting) {
            endstopMax.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::SlowApproach) {
            if (!endstopMax.isTriggered()) {
                endstopMax.simulateTrigger(true);
            } else {
                endstopMax.simulateTrigger(false);
                axis.stop();
            }
        }
    });

    bool homeMaxSuccess = homing.home(axis, cfg);
    printTestResult("Two-stage homing Max completes successfully", homeMaxSuccess);
    printTestResult("Axis position matches homePosition (+180 deg)", axis.getPosition() == 180.0f);

    // Limit protection
    axis.setPosition(0.0f);
    axis.enableLimitProtectionMin(true);
    axis.enableLimitProtectionMax(true);
    printTestResult("Limit protection min enabled", axis.isLimitProtectionMinEnabled());
    printTestResult("Limit protection max enabled", axis.isLimitProtectionMaxEnabled());

    axis.run(-10.0f, 50.0f, 100.0f, 100.0f);
    printTestResult("Driver is running before Min limit hit", driver.isRunning());
    endstopMin.simulateTrigger(true);
    printTestResult("Driver stopped when Min limit hit", !driver.isRunning());

    printTestResult("Move into triggered Min (CW) is blocked", !axis.isDirectionAllowed(Stepper::Direction::Clockwise));
    printTestResult("Move away from triggered Min (CCW) is allowed", axis.isDirectionAllowed(Stepper::Direction::Counterclockwise));

    Stepper::Generic::RotaryAxis::RotaryAxisTask blockedTask;
    blockedTask.angle = -10.0f;
    blockedTask.velocity = 50.0f;
    blockedTask.acceleration = 100.0f;
    blockedTask.deceleration = 100.0f;
    printTestResult("RotaryAxis::run() rejects move into triggered Min", !axis.run(blockedTask));

    Stepper::Generic::RotaryAxis::RotaryAxisTask backoffTask = blockedTask;
    backoffTask.angle = 10.0f;
    printTestResult("RotaryAxis::run() allows back-off from triggered Min", axis.run(backoffTask));
    endstopMin.simulateTrigger(true);
    printTestResult("Min bounce while backing off (CCW) does not stop driver", driver.isRunning());
    endstopMin.simulateTrigger(false);
    axis.stop();

    endstopMax.simulateTrigger(true);
    printTestResult("Move into triggered Max (CCW) is blocked", !axis.isDirectionAllowed(Stepper::Direction::Counterclockwise));
    printTestResult("Move away from triggered Max (CW) is allowed", axis.isDirectionAllowed(Stepper::Direction::Clockwise));
    endstopMax.simulateTrigger(false);
}

void testDualEndstopHoming() {
    std::cout << "\n=== Testing Dual-Endstop (MinThenMax) Homing ===" << std::endl;

    Stepper::Mock::Pin pinEnable(1);
    Stepper::Mock::Pin pinStep(2);
    Stepper::Mock::Pin pinDir(3);
    Stepper::Mock::Driver driver(pinEnable, pinStep, pinDir, 1);
    Stepper::Mock::Generator generator(driver);
    generator.init();
    driver.enable();
    Stepper::Generic::Motor motor(generator, 200, 1.0f);
    Stepper::Generic::RotaryAxis axis(motor, 360.0f, false);

    Stepper::Mock::Pin pinMin(10);
    Stepper::Mock::Pin pinMax(11);
    Stepper::Mock::Endstop endstopMin(pinMin);
    Stepper::Mock::Endstop endstopMax(pinMax);

    Stepper::Generic::Homing homing;
    Stepper::Generic::Homing::Config cfg;
    cfg.strategy = Stepper::Generic::Homing::Strategy::MinThenMax;
    cfg.fastVelocity = 100.0f;
    cfg.slowVelocity = 20.0f;
    cfg.acceleration = 200.0f;
    cfg.deceleration = 200.0f;
    cfg.retractDistance = 10.0f;
    cfg.backoffDistance = 5.0f;
    cfg.maxTravelDistance = 360.0f;
    cfg.homePosition = 0.0f;

    printTestResult("MinThenMax fails with 0 endstops", !homing.homeMinMax(axis, cfg));
    printTestResult("State is Failed with 0 endstops", homing.getState() == Stepper::Generic::Homing::State::Failed);

    axis.setEndstopMin(&endstopMin);
    printTestResult("MinThenMax fails with only Min endstop", !homing.homeMinMax(axis, cfg));
    printTestResult("State is Failed with only 1 endstop", homing.getState() == Stepper::Generic::Homing::State::Failed);

    axis.setEndstopMax(&endstopMax);

    int slowLatchCount = 0;
    generator.setWaitHook([&](Stepper::State) {
        auto hState = homing.getState();
        bool homingMax = (slowLatchCount >= 1);
        auto& activeEndstop = homingMax ? endstopMax : endstopMin;

        if (hState == Stepper::Generic::Homing::State::BackingOff) {
            activeEndstop.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::FastApproach) {
            if (homingMax && axis.getState() != Stepper::State::Stopped) {
                generator.advanceStateAfterStep(100); // +180.0 deg
            }
            activeEndstop.simulateTrigger(true);
        } else if (hState == Stepper::Generic::Homing::State::Retracting) {
            activeEndstop.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::SlowApproach) {
            if (!activeEndstop.isTriggered()) {
                activeEndstop.simulateTrigger(true);
                slowLatchCount++;
            } else {
                uint32_t totalSteps = static_cast<uint32_t>(generator.getState().stepsTotal);
                if (totalSteps > 0) {
                    generator.advanceStateAfterStep(totalSteps);
                }
                activeEndstop.simulateTrigger(false);
                axis.stop();
            }
        }
    });

    bool ok = homing.homeMinMax(axis, cfg);
    printTestResult("MinThenMax homing completes successfully", ok);
    printTestResult("Axis is homed after MinThenMax", axis.isHomed());
    printTestResult("Homing state is Done", homing.getState() == Stepper::Generic::Homing::State::Done);
    printTestResult("Min limit calibrated to 0.0 deg", std::abs(axis.getMinAngleDeg() - 0.0f) < 0.001f);
    printTestResult("Max limit calibrated to 180.0 deg", std::abs(axis.getMaxAngleDeg() - 180.0f) < 0.001f);
    printTestResult("Final angle backed off below 180.0 deg", axis.getAngleDeg() < 180.0f && axis.getAngleDeg() > 170.0f);
    printTestResult("Max endstop released after back-off", !endstopMax.isTriggered());
}

void testLinearAxis() {
    std::cout << "\n=== Testing Linear Axis (mm) & Homing ===" << std::endl;

    Stepper::Mock::Pin pinEnable(1);
    Stepper::Mock::Pin pinStep(2);
    Stepper::Mock::Pin pinDir(3);
    Stepper::Mock::Driver driver(pinEnable, pinStep, pinDir, 1);
    Stepper::Mock::Generator generator(driver);
    generator.init();
    driver.enable();
    Stepper::Generic::Motor motor(generator, 200, 1.0f);
    Stepper::Generic::LinearAxis axis(motor, 8.0f); // 25 steps/mm
    axis.setLimits(0.0f, 200.0f);
    axis.setHomed(true);

    printTestResult("LinearAxis distanceMmPerRevolution == 8.0", axis.getDistanceMmPerRevolution() == 8.0f);
    printTestResult("LinearAxis min limit == 0.0 mm", axis.getMinPositionMm() == 0.0f);
    printTestResult("LinearAxis max limit == 200.0 mm", axis.getMaxPositionMm() == 200.0f);

    axis.setPositionMm(10.0f);
    printTestResult("LinearAxis setPositionMm / getPositionMm", std::abs(axis.getPositionMm() - 10.0f) < 0.01f);

    axis.run(40.0f, 80.0f, 200.0f, 200.0f);
    uint32_t steps = 0;
    while (axis.getState() != Stepper::State::Stopped && steps < 2000) {
        generator.advanceStateAfterStep(1);
        steps++;
    }
    printTestResult("LinearAxis 40 mm move executes 1000 steps", steps == 1000);
    printTestResult("LinearAxis position after 40 mm forward == 50.0 mm", std::abs(axis.getPositionMm() - 50.0f) < 0.01f);

    axis.run(200.0f, 80.0f, 200.0f, 200.0f);
    printTestResult("LinearAxis clamps move exceeding maxLimit (150 mm = 3750 steps)", generator.getState().stepsTotal == 3750);
    axis.stop();

    axis.run(-100.0f, 80.0f, 200.0f, 200.0f);
    printTestResult("LinearAxis clamps move below minLimit (-50 mm = 1250 steps, Backward)",
                    generator.getState().stepsTotal == 1250 && generator.getState().targetDirection == Stepper::Direction::Backward);
    axis.stop();

    Stepper::Mock::Pin pinMin(10);
    Stepper::Mock::Pin pinMax(11);
    Stepper::Mock::Endstop endstopMin(pinMin);
    Stepper::Mock::Endstop endstopMax(pinMax);
    axis.setEndstops(&endstopMin, &endstopMax);

    endstopMin.simulateTrigger(true);
    printTestResult("LinearAxis blocks Backward into triggered Min", !axis.isDirectionAllowed(Stepper::Direction::Backward));
    printTestResult("LinearAxis allows Forward away from triggered Min", axis.isDirectionAllowed(Stepper::Direction::Forward));
    endstopMin.simulateTrigger(false);

    Stepper::Generic::Homing homing;
    Stepper::Generic::Homing::Config cfg;
    cfg.strategy = Stepper::Generic::Homing::Strategy::MinThenMax;
    cfg.fastVelocity = 40.0f;
    cfg.slowVelocity = 8.0f;
    cfg.acceleration = 200.0f;
    cfg.deceleration = 200.0f;
    cfg.retractDistance = 4.0f;
    cfg.backoffDistance = 2.0f;
    cfg.maxTravelDistance = 300.0f;
    cfg.homePosition = 0.0f;

    int slowLatches = 0;
    generator.setWaitHook([&](Stepper::State) {
        auto hState = homing.getState();
        bool homingMax = (slowLatches >= 1);
        auto& active = homingMax ? endstopMax : endstopMin;
        if (hState == Stepper::Generic::Homing::State::BackingOff) {
            active.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::FastApproach) {
            if (homingMax && axis.getState() != Stepper::State::Stopped) {
                generator.advanceStateAfterStep(2500); // 100.0 mm
            }
            active.simulateTrigger(true);
        } else if (hState == Stepper::Generic::Homing::State::Retracting) {
            active.simulateTrigger(false);
            axis.stop();
        } else if (hState == Stepper::Generic::Homing::State::SlowApproach) {
            if (!active.isTriggered()) {
                active.simulateTrigger(true);
                slowLatches++;
            } else {
                uint32_t total = static_cast<uint32_t>(generator.getState().stepsTotal);
                if (total > 0) {
                    generator.advanceStateAfterStep(total);
                }
                active.simulateTrigger(false);
                axis.stop();
            }
        }
    });

    bool homed = homing.homeMinMax(axis, cfg);
    printTestResult("LinearAxis MinThenMax homing succeeds", homed);
    printTestResult("LinearAxis calibrated max limit == 100.0 mm", std::abs(axis.getMaxPositionMm() - 100.0f) < 0.01f);
    printTestResult("LinearAxis backed off to 98.0 mm", std::abs(axis.getPositionMm() - 98.0f) < 0.01f);
}

void testStoppingModes() {
    std::cout << "\n=== Testing Stopping Modes & EmergencyStop Latch ===" << std::endl;

    Stepper::Mock::Pin pin(0);
    Stepper::Mock::Driver driver(pin, pin, pin, 1);
    Stepper::Mock::Generator generator(driver);
    generator.init();
    Stepper::Generic::Motor motor(generator, 200, 1.0f);
    Stepper::Generic::LinearAxis axis(motor, 5.0f);

    Stepper::Generic::LinearAxis::LinearAxisTask relTask;
    relTask.moveRelative = true;
    relTask.position = 25.0f;
    relTask.velocity = 10.0f;
    relTask.acceleration = 20.0f;
    relTask.deceleration = 20.0f;

    Stepper::Generic::LinearAxis::LinearAxisTask absTask = relTask;
    absTask.moveRelative = false;

    // 1. Disabled driver blocks run()
    printTestResult("Disabled driver rejects Axis::run()", !axis.run(relTask) && !driver.isRunning());

    driver.enable();
    axis.setHomed(true);
    axis.setLimits(-100.0f, 100.0f);

    // 2. pause() + resume() from Paused
    axis.run(relTask);
    for (int i = 0; i < 60 && driver.isRunning(); ++i) {
        driver.stepSingleBatch();
    }
    bool pausedOk = axis.pause() && driver.stepLoop() && axis.isPaused() && !driver.isRunning();
    printTestResult("pause() decelerates to State::Paused", pausedOk);
    bool resumedOk = axis.resume() && driver.isRunning() && driver.stepLoop() &&
                     axis.getState() == Stepper::State::Stopped &&
                     std::abs(axis.getPositionMm() - 25.0f) < 0.03f;
    printTestResult("resume() completes exact 25.0 mm move", resumedOk);

    // 3. resume() mid-deceleration into pause
    axis.setPositionMm(0.0f);
    axis.run(relTask);
    for (int i = 0; i < 50; ++i) {
        driver.stepSingleBatch();
    }
    bool midPauseOk = axis.pause() && driver.stepSingleBatch() &&
                      axis.getState() == Stepper::State::Decelerating &&
                      axis.resume() && driver.stepLoop() &&
                      std::abs(axis.getPositionMm() - 25.0f) < 0.03f;
    printTestResult("resume() during pause deceleration finishes move", midPauseOk);

    // 4. fastStop()
    axis.setFastStopDeceleration(100.0f);
    axis.run(relTask);
    for (int i = 0; i < 50; ++i) {
        driver.stepSingleBatch();
    }
    axis.fastStop();
    bool fastStopOk = driver.isRunning() && driver.stepLoop() &&
                      axis.getState() == Stepper::State::Stopped &&
                      axis.isHomed() && !axis.resume();
    printTestResult("fastStop() decelerates to Stopped, keeps isHomed(), not resumable", fastStopOk);

    // 5. emergencyStop(true) latch & clearEmergencyStop(false/true)
    axis.run(relTask);
    driver.stepSingleBatch();
    axis.emergencyStop(true);
    bool estopLatched = axis.isEmergencyStopped() && !driver.isRunning() && !driver.isEnabled() && !axis.isHomed();
    axis.stop(); // must not clear E-stop
    printTestResult("emergencyStop(true) latches EmergencyStop and disables driver", estopLatched && axis.isEmergencyStopped());

    driver.enable();
    printTestResult("Latched EmergencyStop blocks relative run() even if driver enabled", !axis.run(relTask));

    driver.disable();
    bool clearNoEnable = axis.clearEmergencyStop(false) && !axis.isEmergencyStopped() &&
                         axis.getState() == Stepper::State::Stopped && !driver.isEnabled() && !axis.run(relTask);
    printTestResult("clearEmergencyStop(false) clears latch, keeps driver disabled", clearNoEnable);

    driver.enable();
    axis.run(relTask);
    driver.stepSingleBatch();
    axis.emergencyStop(true);
    bool clearWithEnable = axis.clearEmergencyStop(true) && driver.isEnabled() &&
                           !axis.isHomed() && !axis.run(absTask) && axis.run(relTask);
    printTestResult("clearEmergencyStop(true) re-enables driver, keeps abs blocked until homed", clearWithEnable);
}

void testStopController() {
    std::cout << "\n=== Testing StopController (Axes & Standalone Motor) ===" << std::endl;

    Stepper::Mock::Pin p0(0), p1(1), p2(2);
    Stepper::Mock::Driver d0(p0, p0, p0), d1(p1, p1, p1), d2(p2, p2, p2);
    Stepper::Mock::Generator g0(d0), g1(d1), g2(d2);
    g0.init();
    g1.init();
    g2.init();
    d0.enable();
    d1.enable();
    d2.enable();

    Stepper::Generic::Motor m0(g0, 200, 1.0f), m1(g1, 200, 1.0f), standaloneMotor(g2, 200, 1.0f);
    Stepper::Generic::LinearAxis linearAxis(m0, 5.0f);
    Stepper::Generic::RotaryAxis rotaryAxis(m1, 360.0f, false);
    linearAxis.setHomed(true);
    linearAxis.setLimits(-100.0f, 100.0f);
    rotaryAxis.setHomed(true);
    rotaryAxis.setLimits(-360.0f, 360.0f);

    Stepper::Generic::StopController controller;
    bool regOk = !controller.registerTarget(-1, linearAxis) &&
                 controller.registerTarget(0, linearAxis) &&
                 controller.registerTarget(1, rotaryAxis) &&
                 controller.registerTarget(2, standaloneMotor) &&
                 !controller.registerTarget(1, standaloneMotor) &&
                 controller.targetCount() == 3;
    printTestResult("StopController validates IDs and rejects duplicates", regOk);

    linearAxis.run(25.0f, 10.0f, 20.0f, 20.0f);
    rotaryAxis.run(180.0f, 90.0f, 180.0f, 180.0f);
    standaloneMotor.run(5.0f, 2.0f, 4.0f, 4.0f);
    for (int i = 0; i < 40; ++i) {
        d0.stepSingleBatch();
        d1.stepSingleBatch();
        d2.stepSingleBatch();
    }

    bool targetedPause = controller.pause(2) && d2.stepLoop() &&
                         standaloneMotor.isPaused() && !linearAxis.isPaused() && !rotaryAxis.isPaused() &&
                         d0.isRunning() && d1.isRunning();
    printTestResult("Targeted pause(motorId) pauses only standalone Motor", targetedPause);

    bool targetedResume = controller.resume(2) && d2.stepLoop() &&
                          standaloneMotor.getState() == Stepper::State::Stopped;
    printTestResult("Targeted resume(motorId) finishes Motor move", targetedResume);

    controller.pauseAll();
    bool pauseAllOk = d0.stepLoop() && d1.stepLoop() && linearAxis.isPaused() && rotaryAxis.isPaused();
    controller.resumeAll();
    bool resumeAllOk = d0.isRunning() && d1.isRunning();
    printTestResult("pauseAll() and resumeAll() affect all targets", pauseAllOk && resumeAllOk);

    controller.emergencyStopAll(true);
    bool estopAllOk = linearAxis.isEmergencyStopped() && rotaryAxis.isEmergencyStopped() &&
                      standaloneMotor.isEmergencyStopped() && !d0.isEnabled() && !d1.isEnabled() && !d2.isEnabled();
    controller.clearEmergencyStopAll(true);
    bool clearAllOk = !linearAxis.isEmergencyStopped() && !rotaryAxis.isEmergencyStopped() &&
                      !standaloneMotor.isEmergencyStopped() && d0.isEnabled() && d1.isEnabled() && d2.isEnabled();
    printTestResult("emergencyStopAll(true) + clearEmergencyStopAll(true)", estopAllOk && clearAllOk);

    bool unregOk = controller.unregisterTarget(0) && !controller.unregisterTarget(0) &&
                   controller.unregisterTarget(rotaryAxis) && controller.unregisterTarget(2) &&
                   controller.targetCount() == 0;
    printTestResult("unregisterTarget by ID and by reference", unregOk);
}

void testDeferredInitAndOptionalEnablePin() {
    std::cout << "\n=== Testing Deferred init() & Optional/Shared enablePin ===" << std::endl;

    Stepper::Mock::Pin stepPin(1), dirPin(2);
    Stepper::Mock::Driver drvNullEn(nullptr, stepPin, dirPin, 1);
    Stepper::Mock::Generator gen(drvNullEn);

    printTestResult("Generator ctor does not call driver.init()", drvNullEn.initCalls_ == 0);
    drvNullEn.initShouldSucceed_ = false;
    bool failProp = !gen.init() && drvNullEn.initCalls_ == 1;
    drvNullEn.initShouldSucceed_ = true;
    bool passProp = gen.init() && drvNullEn.initCalls_ == 2;
    printTestResult("Generator::init() propagates driver.init() result", failProp && passProp);

    bool nullEnOk = !drvNullEn.hasEnablePin() && drvNullEn.isEnabled() &&
                    gen.run(100, 100.0f, 100.0f, 100.0f, Stepper::Direction::Positive);
    printTestResult("Driver with nullptr enablePin starts enabled and runs", nullEnOk);

    Stepper::Mock::Pin sharedEn(10), s1(11), d1Pin(12), s2(13), d2Pin(14);
    Stepper::Mock::Driver sharedA(&sharedEn, s1, d1Pin, 1);
    Stepper::Mock::Driver sharedB(&sharedEn, s2, d2Pin, 1);
    sharedA.enable();
    bool sharedEnOn = sharedA.isEnabled() && sharedB.isEnabled() && sharedEn.isEnabled();
    sharedB.disable();
    bool sharedEnOff = !sharedA.isEnabled() && !sharedB.isEnabled() && !sharedEn.isEnabled();
    printTestResult("Shared Generic::Pin* enablePin toggles both drivers", sharedEnOn && sharedEnOff);
}

int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "   ESP32 Modular Stepper Pure C++ Sandbox " << std::endl;
    std::cout << "==========================================" << std::endl;

    testEnumHelpers();
    testFixedPointSquareroot();
    testStepperGeneratorKinematics();
    testStepperEndstop();
    testStepCountExactness();
    testRotaryAxisHoming();
    testDualEndstopHoming();
    testLinearAxis();
    testStoppingModes();
    testStopController();
    testDeferredInitAndOptionalEnablePin();

    std::cout << "\n------------------------------------------" << std::endl;
    std::cout << "Passed: " << g_passedTests << " | Failed: " << g_failedTests << std::endl;
    return (g_failedTests == 0) ? 0 : 1;
}
