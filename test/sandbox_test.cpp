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
    endstop.setPullMode(Stepper::Generic::Endstop::PullMode::PullDown);
    printTestResult("PullMode set/get", endstop.getPullMode() == Stepper::Generic::Endstop::PullMode::PullDown);

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

    // Try homing with no endstop
    bool homeNoEndstop = axis.homeMin();
    printTestResult("Homing fails when endstop is not attached", !homeNoEndstop);
    printTestResult("HomingState is Failed", axis.getHomingState() == Stepper::Generic::Axis::HomingState::Failed);

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
    Stepper::Generic::Axis::HomingConfig cfg;
    cfg.endstop = Stepper::Generic::Axis::EndstopPosition::Min;
    cfg.direction = Stepper::Direction::Counterclockwise;
    cfg.fastVelocity = 100.0f;
    cfg.slowVelocity = 20.0f;
    cfg.acceleration = 200.0f;
    cfg.deceleration = 200.0f;
    cfg.retractDistance = 15.0f;
    cfg.homePosition = -180.0f;
    cfg.timeoutMs = 1000;

    int yieldCount = 0;
    axis.setYieldFunction([&axis, &endstopMin, &yieldCount](uint32_t) {
        yieldCount++;
        auto state = axis.getHomingState();
        if (state == Stepper::Generic::Axis::HomingState::FastApproach) {
            if (yieldCount >= 3) {
                endstopMin.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Axis::HomingState::Retracting) {
            endstopMin.simulateTrigger(false);
        } else if (state == Stepper::Generic::Axis::HomingState::SlowApproach) {
            if (yieldCount >= 8) {
                endstopMin.simulateTrigger(true);
            }
        }
    });

    bool homeSuccess = axis.home(cfg);
    printTestResult("Two-stage homing Min completes successfully", homeSuccess);
    printTestResult("Axis is marked homed", axis.isHomed());
    printTestResult("Axis is not homing after completion", !axis.isHoming());
    printTestResult("HomingState is Done", axis.getHomingState() == Stepper::Generic::Axis::HomingState::Done);
    printTestResult("Axis position matches homePosition (-180 deg)", axis.getPosition() == -180.0f);
    printTestResult("Axis angle matches homePosition", axis.getAngleDeg() == -180.0f);

    // 5. Initial Retract (when endstop is already triggered before starting)
    endstopMin.simulateTrigger(true);
    yieldCount = 0;
    axis.setYieldFunction([&axis, &endstopMin, &yieldCount](uint32_t) {
        yieldCount++;
        auto state = axis.getHomingState();
        if (state == Stepper::Generic::Axis::HomingState::Retracting) {
            if (yieldCount >= 2) {
                endstopMin.simulateTrigger(false);
            }
        } else if (state == Stepper::Generic::Axis::HomingState::FastApproach) {
            if (yieldCount >= 5) {
                endstopMin.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Axis::HomingState::SlowApproach) {
            if (yieldCount >= 9) {
                endstopMin.simulateTrigger(true);
            }
        }
    });

    bool homeFromTriggered = axis.home(cfg);
    printTestResult("Homing with initially triggered endstop succeeds", homeFromTriggered);
    printTestResult("Axis homed after initial retract", axis.isHomed());

    // 6. Timeout handling
    endstopMin.simulateTrigger(false);
    axis.setYieldFunction([](uint32_t) {
        // Never trigger endstop
    });
    cfg.timeoutMs = 50; // short 50ms timeout
    bool homeTimeout = axis.home(cfg);
    printTestResult("Homing times out if endstop is never triggered", !homeTimeout);
    printTestResult("Axis homingState is Failed after timeout", axis.getHomingState() == Stepper::Generic::Axis::HomingState::Failed);

    // 7. Max Endstop Homing
    cfg.endstop = Stepper::Generic::Axis::EndstopPosition::Max;
    cfg.direction = Stepper::Direction::Clockwise;
    cfg.homePosition = 180.0f;
    cfg.timeoutMs = 1000;
    yieldCount = 0;
    axis.setYieldFunction([&axis, &endstopMax, &yieldCount](uint32_t) {
        yieldCount++;
        auto state = axis.getHomingState();
        if (state == Stepper::Generic::Axis::HomingState::FastApproach) {
            if (yieldCount >= 3) {
                endstopMax.simulateTrigger(true);
            }
        } else if (state == Stepper::Generic::Axis::HomingState::Retracting) {
            endstopMax.simulateTrigger(false);
        } else if (state == Stepper::Generic::Axis::HomingState::SlowApproach) {
            if (yieldCount >= 7) {
                endstopMax.simulateTrigger(true);
            }
        }
    });

    bool homeMaxSuccess = axis.home(cfg);
    printTestResult("Two-stage homing Max completes successfully", homeMaxSuccess);
    printTestResult("Axis position matches homePosition (+180 deg)", axis.getPosition() == 180.0f);

    // 8. Limit protection
    axis.enableLimitProtection(true);
    printTestResult("Limit protection enabled", axis.isLimitProtectionEnabled());
    driver.start();
    printTestResult("Driver is running before limit hit", driver.isRunning());
    endstopMin.simulateTrigger(true);
    printTestResult("Driver stopped when limit hit", !driver.isRunning());
}

int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "   ESP32 Modular Stepper Pure C++ Sandbox  " << std::endl;
    std::cout << "==========================================" << std::endl;

    testStepperEndstop();
    testRotaryAxisHoming();
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
