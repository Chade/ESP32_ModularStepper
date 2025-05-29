#ifndef STEPPER_GENERATOR_H
#define STEPPER_GENERATOR_H

#include "StepperTask.h"

/*
 * The typical chain of a command:
 * The User is asking the axis to move a certain distance
 * The axis calculates with its params the number of revolutions for the motor to turn
 * The axis asks the motor to turn some revolutions
 * The motor alculates with its params the number of ticks to step.
 * The motor could ask the driver to execute the number of ticks - BUT
 * we want a smooth operation, so the motor has to formulate a task for the generator
 * The generator decides, if there will be a ramp or continuus operations or standstill
 * The generator then creates the ticks the driver translates to pulses for the real HW
 */

namespace Stepper
{
  class Generator
  {
    public:
      Generator();
      ~Generator();

      void processTask(const Task& task);
      uint64_t calcNextStepPeriod();

    private:
      int64_t  m_currentPosition;
      uint16_t m_currentVelocity;
      uint16_t m_currentAcceleration;
      Direction m_currentDirection;

      uint16_t m_cutoffVelocity;
  };
}


#endif // STEPPER_GENERATOR_H