#include <iostream>
#include <cassert>
#include <iomanip>
#include <chrono>
#include <random>
#include <vector>

// Include project headers using host stubs
#include "StepperGenerator.hpp"
#include "StepperDriver_Base.hpp"
#include "StepperHelper.hpp"
#include <FixedPoints.h>
#include <FixedPointsCommon.h>

using UQ20x12 = UFixed<20,12>;

// Helper print function for test assertions
void printTestResult(const std::string& testName, bool success) {
    if (success) {
        std::cout << "[PASS] " << testName << std::endl;
    } else {
        std::cout << "[FAIL] " << testName << std::endl;
    }
}

// Minimal mock driver implementation for host sandbox testing
class MockDriver : public Stepper::DriverBase {
public:
    MockDriver() : DriverBase(1, 2, 3, 1) {}
    void init() override {}
    void start() override {}
    void stop() override {}
    bool isRunning() override { return false; }
    void update(uint32_t stepsDone, uint32_t stepsToDo, float pulsePeriodNew, float pulsePeriodIncrement) override {}
};

void printGeneratorState(const Stepper::Generator::GeneratorState& state) {
    std::cout << "Generator State: " << static_cast<int>(state.currentState)  << " | " << static_cast<int>(state.previousState) << std::endl;
    std::cout << "  Direction: " << static_cast<int>(state.currentDirection) << " | " << static_cast<int>(state.targetDirection) << std::endl;
    std::cout << "  Velocity: " << static_cast<float>(state.currentVelocity) << " steps/s" << " --> " << static_cast<float>(state.targetVelocity) << " steps/s" << std::endl;
    std::cout << "  Acceleration: " << static_cast<float>(state.acceleration) << " steps/s^2" << "  Deceleration: " << static_cast<float>(state.deceleration) << " steps/s^2" << std::endl;
    std::cout << "  Steps: " << state.stepsDone << "/" << state.stepsTotal << " [" << state.stepsAcc << "|" << state.stepsConst << "|" << state.stepsDec << "]" << std::endl;
}

void testStepperGenerator() {
    std::cout << "\n=== Testing Stepper Generator ===" << std::endl;

    MockDriver driver;
    Stepper::Generator generator(driver);

    // Test position mode task (steps > 0)
    Stepper::Generator::GeneratorTask task;
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

void testFixedPointKinematics() {
    std::cout << "\n=== Testing FixedPoints Kinematics Math ===" << std::endl;

    UQ20x12 v0 = 0.0;
    UQ20x12 accel = 1000.5; // steps/s^2
    UQ20x12 dt = 0.01;      // 10ms

    UQ20x12 v1 = v0 + accel * dt;
    std::cout << "Calculated velocity after 10ms: " << static_cast<float>(v1) << " steps/s" << std::endl;

    // Check fixed-point quantization result (~9.77 steps/s due to 12-bit fractional resolution)
    bool checkFP = (static_cast<float>(v1) > 9.5f && static_cast<float>(v1) < 10.5f);
    printTestResult("FixedPoint Velocity Accumulation", checkFP);
}

void testFixedPointSquareroot() {
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
    MockDriver driver;
    Stepper::Generator generator(driver);

    Stepper::Generator::GeneratorTask task;
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

    Stepper::Generator::GeneratorState state;
    state.stepsTotal = 1249;
    state.currentVelocity = 500.0f;
    state.targetVelocity = 0.0f;
    state.acceleration = 100.0f;
    state.deceleration = 100.0f;
    state.currentDirection = Stepper::Direction::Clockwise;
    state.targetDirection = Stepper::Direction::Clockwise;

    Stepper::Generator::computeStepProfile(state);
    printGeneratorState(state);

    Stepper::Generator::computeStepProfile_v2(state);
    printGeneratorState(state);
}

int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "   ESP32 Modular Stepper Pure C++ Sandbox  " << std::endl;
    std::cout << "==========================================" << std::endl;

    //testEnumHelpers();
    testStepperGenerator();
    testBatchKinematics();
    testFixedPointKinematics();
    testFixedPointSquareroot();
    testStepProfileGeneration();

    std::cout << "\nAll sandbox tests completed successfully!\n" << std::endl;
    return 0;
}
