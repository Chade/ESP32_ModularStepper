#ifndef STEPPER_HELPER_H
#define STEPPER_HELPER_H

#include <type_traits>

namespace Stepper
{
  template <typename T> auto cast_enum_to_base(T e)
  {
    return static_cast<std::underlying_type_t<T>>(e);
  };

}

#endif // STEPPER_HELPER_H