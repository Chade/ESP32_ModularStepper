# ESP32_ModularStepper

A modular, hardware-accelerated stepper motor control library for ESP32 using MCPWM pulse generation, fixed-point (`UQ20x12`) trapezoidal/triangular velocity profiles, and FreeRTOS task/event-loop integration.

## Layered Architecture

Each layer depends on the layer below it via clean abstractions (`Stepper::Generic`) with hardware implementations under `Stepper::ESP32`:

```
Pin -> Driver (MCPWM) -> Generator (UQ20x12 kinematics) -> Motor -> Axis (LinearAxis / RotaryAxis)
                                                                       ^
Endstop (debounced GPIO ISR) ------------------------------------------+
Homing (single / dual endstop, sync & async FreeRTOS task)
StopController / ESP32::Event (central pause / resume / fastStop / emergencyStop + latch)
Assembly (MotorAssembly, LinearAxisAssembly, RotaryAxisAssembly)
```

- **Small modular units**: Use `MotorAssembly::create(motorCfg)` for a standalone `Pin -> Driver -> Generator -> Motor` stack, or `LinearAxisAssembly::create(axisCfg)` / `RotaryAxisAssembly::create(axisCfg)` for full axes with optional endstops and soft limits.
- **Optional or shared enable pin**: Set `DriverConfig::hasEnablePin = false` for hardwired `EN`, or pass a shared `Generic::Pin*` to `create(..., sharedEnablePin)` to share one `EN` line across multiple drivers.
- **Stopping & recovery**:
  - `pause()` / `resume()`: Decelerates with the task's deceleration to `State::Paused` and resumes the remaining distance.
  - `fastStop()`: Decelerates with configurable `fastStopDeceleration` to `State::Stopped`, preserving homing.
  - `emergencyStop(disableDriver)`: Immediate stop into a latched `State::EmergencyStop` (optionally disabling the driver and clearing `isHomed()`), requiring `clearEmergencyStop(enableDriver)` before new moves can run.
- **Centralized stop control**: Register axes and standalone motors with `Stepper::Generic::StopController` (manually or via `config.id >= 0`), and optionally attach it to the ESP32 stop event loop via `stopEventLoopAttachController(controller)` for ISR-safe `stopEventPostISR(...)` triggers.

## Dependencies

- **Required**: [`FixedPoints`](https://github.com/Pharap/FixedPointsArduino) (`FixedPoints` 1.1.2+)
- **Example-only**: `ESP32RotaryEncoder` (required only by `examples/LinearAxisEncoder`)

## Installation

### Arduino IDE / Arduino CLI
1. Install `FixedPoints` via the Library Manager (or `arduino-cli lib install FixedPoints`).
2. Clone or copy this repository into your Arduino `libraries/` folder as `ESP32_ModularStepper`:
   ```bash
   git clone https://github.com/Chade/ESP32_ModularStepper.git ~/Arduino/libraries/ESP32_ModularStepper
   ```
3. Include the umbrella header in your sketch:
   ```cpp
   #include <ESP32_ModularStepper.h>
   ```

## Quick Start

```cpp
#include <ESP32_ModularStepper.h>
#include <memory>

Stepper::Generic::StopController stopController;
std::unique_ptr<Stepper::ESP32::LinearAxisAssembly> xAxis;

void setup() {
  Stepper::Generic::LinearAxisConfig cfg;
  cfg.id = 0; // Auto-register with StopController
  cfg.distanceMmPerRevolution = 5.0f;

  cfg.motor.driver.enablePin    = {25, false, Stepper::Generic::Pin::IoMode::Output, Stepper::Generic::Pin::PullMode::Floating};
  cfg.motor.driver.stepPin      = {26, false, Stepper::Generic::Pin::IoMode::Output, Stepper::Generic::Pin::PullMode::Floating};
  cfg.motor.driver.directionPin = {27, false, Stepper::Generic::Pin::IoMode::Output, Stepper::Generic::Pin::PullMode::Floating};
  cfg.motor.driver.microsteps   = 16;
  cfg.motor.stepsPerRevolution  = 200;

  xAxis = Stepper::ESP32::LinearAxisAssembly::create(cfg, &stopController);
  if (!xAxis) {
    return;
  }

  xAxis->driver().enable();
  xAxis->axis().run(20.0f, 10.0f, 50.0f, 50.0f); // +20 mm @ 10 mm/s
}

void loop() {
  delay(100);
}
```

## Examples

- **`examples/BasicLinearAxis`**: Minimal self-contained linear axis setup using `LinearAxisAssembly`, `StopController`, and stopping modes (`pause`, `resume`, `fastStop`, `emergencyStop`).
- **`examples/LinearAxisEncoder`**: Full interactive example with `ESP32RotaryEncoder`, dual endstops (`Min` / `Max`), and asynchronous `MinThenMax` homing.

## Running Host Unit Tests

Pure C++ unit tests for `Generic/` components (kinematics, endstops, homing, stopping modes, E-stop latch, `StopController`) run on the host without ESP32 hardware:

```bash
./test/run_sandbox.sh
```
