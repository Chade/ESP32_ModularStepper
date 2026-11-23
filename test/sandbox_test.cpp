#include <iostream>
#include <cassert>
#include <iomanip>
#include <chrono>
#include <random>
#include <vector>

#include <FixedPoints.h>
#include <FixedPointsCommon.h>
// Include project headers using host stubs
#include "Generic/Generator.hpp"
#include "StepperDriver_Mock.hpp"
#include "StepperEndstop_Mock.hpp"
#include "Generic/Helper.hpp"
#include "Generic/Motor.hpp"
#include "Generic/Axis.hpp"
#include "Generic/Axis_Rotary.hpp"
#include "Generic/Axis_Linear.hpp"
#include "Generic/Homing.hpp"

using UQ20x12 = UFixed<20,12>;

// Helper print function for test assertions
void printTestResult(const std::string& testName, bool success) {
    if (success) {
        std::cout << "[PASS] " << testName << std::endl;
    } else {
        std::cout << "[FAIL] " << testName << std::endl;
    }
}

void printGeneratorState(const Stepper::Generic::Generator::GeneratorState& state) {
    std::cout << "Generator State: " << static_cast<int>(state.currentState)  << " | " << static_cast<int>(state.previousState) << std::endl;
    std::cout << "  Direction: " << static_cast<int>(state.currentDirection) << " | " << static_cast<int>(state.targetDirection) << std::endl;
    std::cout << "  Velocity: " << static_cast<float>(state.currentVelocity) << " steps/s" << " --> " << static_cast<float>(state.targetVelocity) << " steps/s" << std::endl;
    std::cout << "  Acceleration: " << static_cast<float>(state.acceleration) << " steps/s^2" << "  Deceleration: " << static_cast<float>(state.deceleration) << " steps/s^2" << std::endl;
    std::cout << "  Steps: " << state.stepsDone << "/" << state.stepsTotal << " [" << state.stepsAcc << "|" << state.stepsConst << "|" << state.stepsDec << "]" << std::endl;
}

void testStepperGenerator() {
    std::cout << "\n=== Testing Stepper Generator ===" << std::endl;

    Stepper::Mock::Pin pin(0);
    Stepper::Mock::Driver driver(pin, pin, pin);
    Stepper::Generic::Generator generator(driver);

    // Test position mode task (steps > 0)
    Stepper::Generic::Generator::GeneratorTask task;
    task.steps = 1000;
    task.velocity = 1000.0f;
    task.acceleration = 1000.0f;
    task.deceleration = 1000.0f;
    task.direction = Stepper::Direction::Clockwise;

    generator.run(task);
    printGeneratorState(generator.getState());

    for (uint32_t i = 0; i < task.steps; ++i) {
        generator.advanceStateAfterStep(1);
        printGeneratorState(generator.getState());
    }
}

void testStepperGeneratorZeroAcc() {
    std::cout << "\n=== Testing Stepper Generator With Zero Acc ===" << std::endl;

    Stepper::Mock::Pin pin(0);
    Stepper::Mock::Driver driver(pin, pin, pin);
    Stepper::Generic::Generator generator(driver);

    // Test position mode task (steps > 0)
    Stepper::Generic::Generator::GeneratorTask task;
    task.steps = 20;
    task.velocity = 10.0f;
    task.acceleration = 0.0f;
    task.deceleration = 10.0f;
    task.direction = Stepper::Direction::Clockwise;

    generator.run(task);
    printGeneratorState(generator.getState());

    for (uint32_t i = 0; i < task.steps; ++i) {
        generator.advanceStateAfterStep(1);
        printGeneratorState(generator.getState());
    }
}

void testStepperGeneratorZeroDec() {
    std::cout << "\n=== Testing Stepper Generator With Zero Dec ===" << std::endl;

    Stepper::Mock::Pin pin(0);
    Stepper::Mock::Driver driver(pin, pin, pin);
    Stepper::Generic::Generator generator(driver);

    // Test position mode task (steps > 0)
    Stepper::Generic::Generator::GeneratorTask task;
    task.steps = 20;
    task.velocity = 10.0f;
    task.acceleration = 10.0f;
    task.deceleration = 0.0f;
    task.direction = Stepper::Direction::Clockwise;

    generator.run(task);
    printGeneratorState(generator.getState());

    for (uint32_t i = 0; i < task.steps; ++i) {
        generator.advanceStateAfterStep(1);
        printGeneratorState(generator.getState());
    }
}

void testStepperGeneratorZeroAccDec() {
    std::cout << "\n=== Testing Stepper Generator With Zero Acc & Dec ===" << std::endl;

    Stepper::Mock::Pin pin(0);
    Stepper::Mock::Driver driver(pin, pin, pin);
    Stepper::Generic::Generator generator(driver);

    // Test position mode task (steps > 0)
    Stepper::Generic::Generator::GeneratorTask task;
    task.steps = 20;
    task.velocity = 10.0f;
    task.acceleration = 0.0f;
    task.deceleration = 0.0f;
    task.direction = Stepper::Direction::Clockwise;

    generator.run(task);
    printGeneratorState(generator.getState());

    for (uint32_t i = 0; i < task.steps; ++i) {
        generator.advanceStateAfterStep(1);
        printGeneratorState(generator.getState());
    }
}

// Pure C++ Function Tests
void testEnumHelpers() {
    std::cout << "\n=== Testing StepperHelper Enums ===" << std::endl;

    auto dirVal = Stepper::cast_enum_to_base(Stepper::Direction::Counterclockwise);
    bool checkDir = (dirVal == 1);
    printTestResult("Direction cast_enum_to_base (Counterclockwise == 1)", checkDir);

    auto stateVal = Stepper::cast_enum_to_base(Stepper::State::Accelerating);
    bool checkState = (stateVal == 6);
    printTestResult("State cast_enum_to_base (Accelerating == 6)", checkState);

    auto compared = Stepper::compare_enums(Stepper::State::Accelerating, Stepper::State::Running);
    printTestResult("State bitwise compare_enums", compared == (6 & 2));
}

void testFixedPointSquareroot() {
    std::cout << "\n=== Testing Squareroot Calculation ===" << std::endl;
    UQ20x12 v1 = 2.0;
    UQ20x12 v2 = 16.0;
    UQ20x12 v3 = 100.0;
    UQ20x12 v4 = 0.25;
    UQ20x12 result;
    
    result = Stepper::sqrt(v1);
    printTestResult("FixedPoint sqrt(2.0)", (result > 1.41 && result < 1.42));

    result = Stepper::sqrt(v2);
    printTestResult("FixedPoint sqrt(16.0)", (result == 4.0));

    result = Stepper::sqrt(v3);
    printTestResult("FixedPoint sqrt(100.0)", (result == 10.0));

    result = Stepper::sqrt(v4);
    printTestResult("FixedPoint sqrt(0.25)", (result == 0.5));

    // Verify 64-bit integer square root edge cases
    printTestResult("isqrt64(0) == 0", Stepper::isqrt64(0) == 0);
    printTestResult("isqrt64(1) == 1", Stepper::isqrt64(1) == 1);
    printTestResult("isqrt64(2) == 1", Stepper::isqrt64(2) == 1);
    printTestResult("isqrt64(3) == 1", Stepper::isqrt64(3) == 1);
    printTestResult("isqrt64(4) == 2", Stepper::isqrt64(4) == 2);
    printTestResult("isqrt64(0xFFFFFFFF) == 65535", Stepper::isqrt64(0xFFFFFFFFULL) == 65535);
    printTestResult("isqrt64(0x100000000) == 65536", Stepper::isqrt64(0x100000000ULL) == 65536);
    printTestResult("isqrt64(UINT64_MAX) == 4294967295", Stepper::isqrt64(UINT64_MAX) == 4294967295ULL);
}

void testBatchKinematics() {
    std::cout << "\n=== Testing Batch Kinematics (Multi-Step Advances) ===" << std::endl;
    Stepper::Mock::Pin pin(0);
    Stepper::Mock::Driver driver(pin, pin, pin);
    Stepper::Generic::Generator generator(driver);

    Stepper::Generic::Generator::GeneratorTask task;
    task.steps = 1000;
    task.velocity = 1000.0f;
    task.acceleration = 1000.0f;
    task.deceleration = 1000.0f;
    task.direction = Stepper::Direction::Clockwise;

    generator.run(task);
    printGeneratorState(generator.getState());

    // Advance 500 steps at a time during acceleration
    generator.advanceStateAfterStep(500);
    float v1 = generator.getVelocity();
    printGeneratorState(generator.getState());

    generator.advanceStateAfterStep(500);
    float v2 = generator.getVelocity();
    printGeneratorState(generator.getState());

    // Kinematics expected: sqrt(2 * 100 * 50) = sqrt(10000) = 100.0 steps/s
    bool checkBatchAcc = (v1 > 999.0f && v1 < 1001.0f && v2 < 1.0);
    printTestResult("Batch Step Acceleration (500 steps)", checkBatchAcc);
}



void testStepProfileGeneration() {
    std::cout << "\n=== Testing Step Profile generation ===" << std::endl;

    Stepper::Generic::Generator::GeneratorState state;
    state.stepsTotal = 1249;
    state.currentVelocity = 500.0f;
    state.targetVelocity = 0.0f;
    state.acceleration = 100.0f;
    state.deceleration = 100.0f;
    state.currentDirection = Stepper::Direction::Clockwise;
    state.targetDirection = Stepper::Direction::Clockwise;

    Stepper::Generic::Generator::computeStepProfile(state);
    printGeneratorState(state);

    Stepper::Generic::Generator::computeStepProfile_v2(state);
    printGeneratorState(state);
}

void testStepperEndstop() {
    std::cout << "\n=== Testing Stepper Endstop ===" << std::endl;

    Stepper::Mock::Pin pin(15);
    Stepper::Mock::Endstop endstop(pin);

    printTestResult("Endstop pin number", endstop.getPin().getPin() == 15);

    // Test 1: Callback for both rising and falling edges
    int risingCount = 0;
    int fallingCount = 0;
    endstop.registerCallback([&risingCount, &fallingCount](Stepper::Generic::Endstop::Edge edge, void* ctx) {
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

    // Test 2: Separate callbacks for rising and falling
    int separateRisingCount = 0;
    int separateFallingCount = 0;
    void* ctxRising = reinterpret_cast<void*>(0x1234);
    void* ctxFalling = reinterpret_cast<void*>(0x5678);
    void* receivedCtxRising = nullptr;
    void* receivedCtxFalling = nullptr;

    endstop.registerCallbackOnRising([&separateRisingCount, &receivedCtxRising](Stepper::Generic::Endstop::Edge edge, void* ctx) {
        separateRisingCount++;
        receivedCtxRising = ctx;
    }, ctxRising);

    endstop.registerCallbackOnFalling([&separateFallingCount, &receivedCtxFalling](Stepper::Generic::Endstop::Edge edge, void* ctx) {
        separateFallingCount++;
        receivedCtxFalling = ctx;
    }, ctxFalling);

    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    printTestResult("Separate callback on rising triggers", separateRisingCount == 1 && separateFallingCount == 0);
    printTestResult("Separate callback receives rising ctx", receivedCtxRising == ctxRising);

    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Falling);
    printTestResult("Separate callback on falling triggers", separateRisingCount == 1 && separateFallingCount == 1);
    printTestResult("Separate callback receives falling ctx", receivedCtxFalling == ctxFalling);

    // Test 3: SimpleCallback overload (void(Edge))
    int simpleCallbackCount = 0;
    endstop.registerCallback([&simpleCallbackCount](Stepper::Generic::Endstop::Edge edge) {
        simpleCallbackCount++;
    });
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Falling);
    printTestResult("SimpleCallback called for both edges", simpleCallbackCount == 2);

    // Test 4: VoidCallback overload (void())
    int voidRisingCount = 0;
    endstop.registerCallbackOnRising([&voidRisingCount]() {
        voidRisingCount++;
    });
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Falling);
    printTestResult("VoidCallback called for rising edge only", voidRisingCount == 1);

    // Test 5: Disable suppresses callbacks
    endstop.disableInterrupt();
    int countWhileDisabled = voidRisingCount;
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    printTestResult("Disabled endstop suppresses callback", voidRisingCount == countWhileDisabled);
    endstop.enableInterrupt();
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    printTestResult("Re-enabled endstop triggers callback", voidRisingCount == countWhileDisabled + 1);

    // Test 6: Unregister callbacks
    endstop.unregisterCallback();
    int countAfterUnregister = voidRisingCount;
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Rising);
    endstop.simulateEdge(Stepper::Generic::Endstop::Edge::Falling);
    printTestResult("Unregistered callback receives no events", voidRisingCount == countAfterUnregister);

    // Test 7: PullMode
    endstop.setPullMode(Stepper::Generic::Pin::PullMode::PullDown);
    printTestResult("PullMode set/get", endstop.getPullMode() == Stepper::Generic::Pin::PullMode::PullDown);

    // Test 8: Debounce
    endstop.setDebounceUs(1000);
    printTestResult("Debounce set/get", endstop.getDebounceUs() == 1000);
}

void testRotaryAxisHoming() {
    std::cout << "\n=== Testing Rotary Axis and Homing ===" << std::endl;

    Stepper::Mock::Pin pinEnable(1);
    Stepper::Mock::Pin pinStep(2);
    Stepper::Mock::Pin pinDir(3);
    Stepper::Mock::Driver driver(pinEnable, pinStep, pinDir);
    Stepper::Generic::Generator generator(driver);
    Stepper::Generic::Motor motor(generator, 200, 1.0f);
    Stepper::Generic::RotaryAxis axis(motor, 360.0f, false);

    // 1. Initial State & limits
    axis.setLimits(-180.0f, 180.0f);
    printTestResult("RotaryAxis min angle", axis.getMinAngleDeg() == -180.0f);
    printTestResult("RotaryAxis max angle", axis.getMaxAngleDeg() == 180.0f);
    printTestResult("RotaryAxis initially not homed", !axis.isHomed());
    printTestResult("RotaryAxis initially not homing", !axis.isHoming());

    // 2. Endstops attachment
    Stepper::Mock::Pin pinMin(10);
    Stepper::Mock::Pin pinMax(11);
    Stepper::Mock::Endstop endstopMin(pinMin);
    Stepper::Mock::Endstop endstopMax(pinMax);

    printTestResult("No min endstop initially", !axis.hasEndstopMin());
    printTestResult("No max endstop initially", !axis.hasEndstopMax());

    Stepper::Generic::Homing homing;

    // Try homing with no endstop
    bool homeNoEndstop = homing.homeMin(axis);
    printTestResult("Homing fails when endstop is not attached", !homeNoEndstop);
    printTestResult("Homing state is Failed", homing.getState() == Stepper::Generic::Homing::State::Failed);

    axis.setEndstopMin(&endstopMin);
    axis.setEndstopMax(&endstopMax);
    printTestResult("Has min endstop after set", axis.hasEndstopMin());
    printTestResult("Has max endstop after set", axis.hasEndstopMax());
    printTestResult("getEndstopMin matches", axis.getEndstopMin() == &endstopMin);
    printTestResult("getEndstopMax matches", axis.getEndstopMax() == &endstopMax);

    // 3. Position tracking
    axis.setPosition(90.0f);
    printTestResult("setPosition / getPosition", axis.getPosition() == 90.0f);
    printTestResult("setPosition / getAngleDeg", axis.getAngleDeg() == 90.0f);

    // 4. Two-Stage Homing Simulation (Min Endstop)
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

    bool sawBackingOff = false;
    int yieldCount = 0;
    homing.setYieldFunction([&axis, &generator, &homing, &endstopMin, &yieldCount, &sawBackingOff](uint32_t) {
        yieldCount++;
        auto state = homing.getState();
        if (state == Stepper::Generic::Homing::State::FastApproach) {
            if (yieldCount >= 3) {
                endstopMin.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Homing::State::Retracting) {
            endstopMin.simulateTrigger(false);
            axis.stop(); // simulate retract move finishing
        } else if (state == Stepper::Generic::Homing::State::SlowApproach) {
            if (yieldCount >= 8) {
                endstopMin.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Homing::State::BackingOff) {
            sawBackingOff = true;
            if (axis.getState() != Stepper::State::Stopped) {
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
    printTestResult("Final back-off phase executed", sawBackingOff);
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

    // 5. Initial Retract (when endstop is already triggered before starting)
    endstopMin.simulateTrigger(true);
    yieldCount = 0;
    homing.setYieldFunction([&axis, &homing, &endstopMin, &yieldCount](uint32_t) {
        yieldCount++;
        auto state = homing.getState();
        if (state == Stepper::Generic::Homing::State::Retracting) {
            if (yieldCount >= 2) {
                endstopMin.simulateTrigger(false);
                axis.stop(); // simulate retract move finishing
            }
        } else if (state == Stepper::Generic::Homing::State::FastApproach) {
            if (yieldCount >= 5) {
                endstopMin.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Homing::State::SlowApproach) {
            if (yieldCount >= 9) {
                endstopMin.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Homing::State::BackingOff) {
            endstopMin.simulateTrigger(false);
            axis.stop();
        }
    });

    bool homeFromTriggered = homing.home(axis, cfg);
    printTestResult("Homing with initially triggered endstop succeeds", homeFromTriggered);
    printTestResult("Axis homed after initial retract", axis.isHomed());

    // 6. Max travel distance abort handling (endstop never triggered)
    endstopMin.simulateTrigger(false);
    yieldCount = 0;
    homing.setYieldFunction([&axis, &yieldCount](uint32_t) {
        yieldCount++;
        if (yieldCount >= 5) {
            // Simulate maxTravelDistance move completing without hitting endstop
            axis.stop();
        }
    });
    bool homeMaxTravelAbort = homing.home(axis, cfg);
    printTestResult("Homing aborts if maxTravelDistance exceeded without trigger", !homeMaxTravelAbort);
    printTestResult("Homing state is Failed after maxTravelDistance abort", homing.getState() == Stepper::Generic::Homing::State::Failed);

    // 7. Max Endstop Homing
    cfg.strategy = Stepper::Generic::Homing::Strategy::Max;
    cfg.homePosition = 180.0f;
    cfg.maxTravelDistance = 360.0f;
    yieldCount = 0;
    homing.setYieldFunction([&axis, &homing, &endstopMax, &yieldCount](uint32_t) {
        yieldCount++;
        auto state = homing.getState();
        if (state == Stepper::Generic::Homing::State::FastApproach) {
            if (yieldCount >= 3) {
                endstopMax.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Homing::State::Retracting) {
            endstopMax.simulateTrigger(false);
            axis.stop(); // simulate retract move finishing
        } else if (state == Stepper::Generic::Homing::State::SlowApproach) {
            if (yieldCount >= 7) {
                endstopMax.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Homing::State::BackingOff) {
            endstopMax.simulateTrigger(false);
            axis.stop();
        }
    });

    bool homeMaxSuccess = homing.home(axis, cfg);
    printTestResult("Two-stage homing Max completes successfully", homeMaxSuccess);
    printTestResult("Axis position matches homePosition (+180 deg)", axis.getPosition() == 180.0f);

    // 8. Limit protection (immediate stop + directional blocking)
    axis.enableLimitProtectionMin(true);
    axis.enableLimitProtectionMax(true);
    printTestResult("Limit protection min enabled", axis.isLimitProtectionMinEnabled());
    printTestResult("Limit protection max enabled", axis.isLimitProtectionMaxEnabled());

    // Moving toward Min -> hitting Min stops driver
    axis.run(10.0f, 50.0f, 100.0f, 100.0f, Stepper::Direction::Clockwise);
    printTestResult("Driver is running before Min limit hit", driver.isRunning());
    endstopMin.simulateTrigger(true);
    printTestResult("Driver stopped when Min limit hit", !driver.isRunning());

    // While Min is triggered: moving further into Min (CW/Backward) is blocked, backing off (CCW/Forward) is allowed
    printTestResult("Move into triggered Min (CW) is blocked", !axis.isDirectionAllowed(Stepper::Direction::Clockwise));
    printTestResult("Move away from triggered Min (CCW) is allowed", axis.isDirectionAllowed(Stepper::Direction::Counterclockwise));

    Stepper::Generic::RotaryAxis::RotaryAxisTask blockedTask;
    blockedTask.angle = 10.0f;
    blockedTask.velocity = 50.0f;
    blockedTask.acceleration = 100.0f;
    blockedTask.deceleration = 100.0f;
    blockedTask.direction = Stepper::Direction::Clockwise;
    printTestResult("RotaryAxis::run() rejects move into triggered Min", !axis.run(blockedTask));

    Stepper::Generic::RotaryAxis::RotaryAxisTask backoffTask = blockedTask;
    backoffTask.direction = Stepper::Direction::Counterclockwise;
    printTestResult("RotaryAxis::run() allows back-off from triggered Min", axis.run(backoffTask));
    // Bounce on Min while backing off in CCW must NOT stop the driver
    endstopMin.simulateTrigger(true);
    printTestResult("Min bounce while backing off (CCW) does not stop driver", driver.isRunning());
    endstopMin.simulateTrigger(false);
    axis.stop();

    // While Max is triggered: moving into Max (CCW/Forward) is blocked, backing off (CW/Backward) is allowed
    endstopMax.simulateTrigger(true);
    printTestResult("Move into triggered Max (CCW) is blocked", !axis.isDirectionAllowed(Stepper::Direction::Counterclockwise));
    printTestResult("Move away from triggered Max (CW) is allowed", axis.isDirectionAllowed(Stepper::Direction::Clockwise));
    endstopMax.simulateTrigger(false);
}

void testStepCountExactness() {
    std::cout << "\n=== Testing Step Count Exactness (90 deg @ 200 steps/rev x 16 microsteps) ===" << std::endl;

    Stepper::Mock::Pin pinEnable(1);
    Stepper::Mock::Pin pinStep(2);
    Stepper::Mock::Pin pinDir(3);
    Stepper::Mock::Driver driver(pinEnable, pinStep, pinDir, 16);
    Stepper::Generic::Generator generator(driver);
    Stepper::Generic::Motor motor(generator, 200, 1.0f);
    Stepper::Generic::RotaryAxis axis(motor, 360.0f, true);

    // Exact homing back-off parameters: 90 deg at 2 deg/s, acc=20 deg/s^2, dec=20 deg/s^2
    axis.setPosition(0.0f);
    axis.run(90.0f, 2.0f, 20.0f, 20.0f, Stepper::Direction::Counterclockwise);

    uint32_t executedSteps = 0;
    while (axis.getState() != Stepper::State::Stopped && executedSteps < 1000) {
        generator.advanceStateAfterStep(1);
        executedSteps++;
    }

    auto state = generator.getState();
    std::cout << "  Profile: [" << state.stepsAcc << "|" << state.stepsConst << "|" << state.stepsDec << "] total=" << state.stepsTotal << std::endl;
    std::cout << "  Executed steps: " << executedSteps << " | positionSteps: " << state.positionSteps
              << " | angle: " << std::fixed << std::setprecision(4) << axis.getAngleDeg() << " deg" << std::endl;

    printTestResult("90 deg slow move executes exactly 800 microsteps", executedSteps == 800);
    printTestResult("90 deg slow move final angle == 90.0 deg", std::abs(axis.getAngleDeg() - 90.0f) < 0.001f);

    // Also verify faster trapezoid move (stepsAcc > 0, stepsDec > 0)
    axis.setPosition(0.0f);
    axis.run(90.0f, 180.0f, 100.0f, 100.0f, Stepper::Direction::Counterclockwise);
    executedSteps = 0;
    while (axis.getState() != Stepper::State::Stopped && executedSteps < 1000) {
        generator.advanceStateAfterStep(1);
        executedSteps++;
    }
    state = generator.getState();
    std::cout << "  Trapezoid profile: [" << state.stepsAcc << "|" << state.stepsConst << "|" << state.stepsDec << "] total=" << state.stepsTotal << std::endl;
    std::cout << "  Executed steps: " << executedSteps << " | angle: " << axis.getAngleDeg() << " deg" << std::endl;
    printTestResult("90 deg trapezoid move executes exactly 800 microsteps", executedSteps == 800);
    printTestResult("90 deg trapezoid move final angle == 90.0 deg", std::abs(axis.getAngleDeg() - 90.0f) < 0.001f);
}

void testDualEndstopHoming() {
    std::cout << "\n=== Testing Dual-Endstop (MinThenMax) Homing ===" << std::endl;

    Stepper::Mock::Pin pinEnable(1);
    Stepper::Mock::Pin pinStep(2);
    Stepper::Mock::Pin pinDir(3);
    Stepper::Mock::Driver driver(pinEnable, pinStep, pinDir, 1);
    Stepper::Generic::Generator generator(driver);
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
    cfg.backoffDistance = 5.0f; // 2 steps = 3.6 deg at 200 steps/rev
    cfg.maxTravelDistance = 360.0f;
    cfg.homePosition = 0.0f;

    // 1. Fails when no endstops attached
    printTestResult("MinThenMax fails with 0 endstops", !homing.homeMinMax(axis, cfg));
    printTestResult("State is Failed with 0 endstops", homing.getState() == Stepper::Generic::Homing::State::Failed);

    // 2. Fails when only Min endstop attached
    axis.setEndstopMin(&endstopMin);
    printTestResult("MinThenMax fails with only Min endstop", !homing.homeMinMax(axis, cfg));
    printTestResult("State is Failed with only 1 endstop", homing.getState() == Stepper::Generic::Homing::State::Failed);

    // 3. Succeeds with both Min and Max endstops attached
    axis.setEndstopMax(&endstopMax);

    // Track whether we are homing Min (phaseGroup=0) or Max (phaseGroup=1)
    int slowLatchCount = 0;
    int yieldCount = 0;
    homing.setYieldFunction([&](uint32_t) {
        yieldCount++;
        auto state = homing.getState();
        bool homingMax = (slowLatchCount >= 1);
        auto& activeEndstop = homingMax ? endstopMax : endstopMin;

        if (state == Stepper::Generic::Homing::State::FastApproach) {
            if (homingMax && axis.getState() != Stepper::State::Stopped) {
                // Simulate traveling 100 steps (+180.0 deg) from Min to Max
                generator.advanceStateAfterStep(100);
                activeEndstop.simulateTrigger(true);
            } else if (!homingMax && yieldCount >= 2) {
                activeEndstop.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Homing::State::Retracting) {
            if (axis.getState() != Stepper::State::Stopped) {
                activeEndstop.simulateTrigger(false);
                axis.stop();
            }
        } else if (state == Stepper::Generic::Homing::State::SlowApproach) {
            if (axis.getState() != Stepper::State::Stopped) {
                activeEndstop.simulateTrigger(true);
                slowLatchCount++;
            }
        } else if (state == Stepper::Generic::Homing::State::BackingOff) {
            if (axis.getState() != Stepper::State::Stopped) {
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
    // 200 steps/rev, 1 microstep, 8.0 mm/rev leadscrew -> 25 steps/mm
    Stepper::Mock::Driver driver(pinEnable, pinStep, pinDir, 1);
    Stepper::Generic::Generator generator(driver);
    Stepper::Generic::Motor motor(generator, 200, 1.0f);
    Stepper::Generic::LinearAxis axis(motor, 8.0f, 0.0f, 200.0f);

    printTestResult("LinearAxis distanceMmPerRevolution == 8.0", axis.getDistanceMmPerRevolution() == 8.0f);
    printTestResult("LinearAxis min limit == 0.0 mm", axis.getMinPositionMm() == 0.0f);
    printTestResult("LinearAxis max limit == 200.0 mm", axis.getMaxPositionMm() == 200.0f);

    // Move 40 mm forward (1000 steps)
    axis.setPositionMm(10.0f);
    printTestResult("LinearAxis setPositionMm / getPositionMm", std::abs(axis.getPositionMm() - 10.0f) < 0.01f);

    axis.run(40.0f, 80.0f, 200.0f, 200.0f, Stepper::Direction::Forward);
    uint32_t steps = 0;
    while (axis.getState() != Stepper::State::Stopped && steps < 2000) {
        generator.advanceStateAfterStep(1);
        steps++;
    }
    printTestResult("LinearAxis 40 mm move executes 1000 steps", steps == 1000);
    printTestResult("LinearAxis position after 40 mm forward == 50.0 mm", std::abs(axis.getPositionMm() - 50.0f) < 0.01f);

    // Soft limit clamping: try moving 200 mm forward from 50 mm (should clamp to 150 mm -> 200 mm max)
    axis.run(200.0f, 80.0f, 200.0f, 200.0f, Stepper::Direction::Forward);
    printTestResult("LinearAxis clamps move exceeding maxLimit (150 mm = 3750 steps)", generator.getState().stepsTotal == 3750);
    axis.stop();

    // Limit protection with Forward / Backward
    Stepper::Mock::Pin pinMin(10);
    Stepper::Mock::Pin pinMax(11);
    Stepper::Mock::Endstop endstopMin(pinMin);
    Stepper::Mock::Endstop endstopMax(pinMax);
    axis.setEndstops(&endstopMin, &endstopMax);

    endstopMin.simulateTrigger(true);
    printTestResult("LinearAxis blocks Backward into triggered Min", !axis.isDirectionAllowed(Stepper::Direction::Backward));
    printTestResult("LinearAxis allows Forward away from triggered Min", axis.isDirectionAllowed(Stepper::Direction::Forward));
    endstopMin.simulateTrigger(false);

    // Dual-endstop homing works identically on LinearAxis
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
    int yields = 0;
    homing.setYieldFunction([&](uint32_t) {
        yields++;
        auto state = homing.getState();
        bool homingMax = (slowLatches >= 1);
        auto& active = homingMax ? endstopMax : endstopMin;
        if (state == Stepper::Generic::Homing::State::FastApproach) {
            if (homingMax && axis.getState() != Stepper::State::Stopped) {
                generator.advanceStateAfterStep(2500); // 100.0 mm
                active.simulateTrigger(true);
            } else if (!homingMax && yields >= 2) {
                active.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Homing::State::Retracting) {
            if (axis.getState() != Stepper::State::Stopped) {
                active.simulateTrigger(false);
                axis.stop();
            }
        } else if (state == Stepper::Generic::Homing::State::SlowApproach) {
            if (axis.getState() != Stepper::State::Stopped) {
                active.simulateTrigger(true);
                slowLatches++;
            }
        } else if (state == Stepper::Generic::Homing::State::BackingOff) {
            if (axis.getState() != Stepper::State::Stopped) {
                uint32_t total = static_cast<uint32_t>(generator.getState().stepsTotal);
                if (total > 0) generator.advanceStateAfterStep(total);
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

int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "   ESP32 Modular Stepper Pure C++ Sandbox  " << std::endl;
    std::cout << "==========================================" << std::endl;

    testStepperEndstop();
    testRotaryAxisHoming();
    testStepCountExactness();
    testDualEndstopHoming();
    testLinearAxis();
    //testEnumHelpers();
    //testStepperGenerator();
    testStepperGeneratorZeroAcc();
    testStepperGeneratorZeroDec();
    testStepperGeneratorZeroAccDec();
    //testBatchKinematics();
    //testFixedPointSquareroot();
    //testStepProfileGeneration();

    std::cout << "\nAll sandbox tests completed successfully!\n" << std::endl;
    return 0;
}
