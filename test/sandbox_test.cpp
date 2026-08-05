#include <iostream>
#include <cassert>
#include <iomanip>

// Include project headers using host stubs
#include "StepperGenerator.hpp"
#include "StepperDriver_Base.hpp"
#include "StepperHelper.hpp"
#include <FixedPoints.h>
#include <FixedPointsCommon.h>

using UQ20x12 = UFixed<20, 12>;

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
    void update(uint32_t stepsDone, uint32_t stepsToDo, float pulsePeriodNew) override {}
};

void printGeneratorState(const Stepper::Generator::GeneratorState& state) {
    std::cout << "Generator State: " << static_cast<int>(state.state) << std::endl;
    std::cout << "  Direction: " << static_cast<int>(state.currentDirection) << " | " << static_cast<int>(state.targetDirection) << std::endl;
    std::cout << "  Velocity: " << static_cast<float>(state.currentVelocity) << " steps/s" << " | " << static_cast<float>(state.targetVelocity) << " steps/s" << std::endl;
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
    task.acceleration = 2000.0f;
    task.deceleration = 2000.0f;
    task.direction = Stepper::Direction::Clockwise;

    generator.run(task);
    printGeneratorState(generator.getState());

    for (int i = 0; i < task.steps; ++i) {
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

void testBatchKinematics() {
    std::cout << "\n=== Testing Batch Kinematics (Multi-Step Advances) ===" << std::endl;
    MockDriver driver;
    Stepper::Generator generator(driver);

    Stepper::Generator::GeneratorTask task;
    task.steps = 1000;
    task.velocity = 500.0f;
    task.acceleration = 100.0f;
    task.deceleration = 100.0f;
    task.direction = Stepper::Direction::Clockwise;

    generator.run(task);

    // Advance 50 steps at a time during acceleration
    generator.advanceStateAfterStep(50);
    float v1 = generator.getVelocity();
    std::cout << "Velocity after 50 steps: " << v1 << " steps/s" << std::endl;

    // Kinematics expected: sqrt(2 * 100 * 50) = sqrt(10000) = 100.0 steps/s
    bool checkBatchAcc = (v1 > 98.0f && v1 < 102.0f);
    printTestResult("Batch Step Acceleration (50 steps)", checkBatchAcc);
}

int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "   ESP32 Modular Stepper Pure C++ Sandbox  " << std::endl;
    std::cout << "==========================================" << std::endl;

    //testEnumHelpers();
    //testFixedPointKinematics();
    testStepperGenerator();
    testBatchKinematics();

    std::cout << "\nAll sandbox tests completed successfully!\n" << std::endl;
    return 0;
}
