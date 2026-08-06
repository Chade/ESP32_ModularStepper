#ifndef STEPPER_HELPER_HPP
#define STEPPER_HELPER_HPP

#include <FixedPoints.h>
#include <type_traits>
#include <cstdint>

using UQ20x12 = UFixed<20, 12>;

namespace Stepper
{
    template <typename T>
    auto cast_enum_to_base(T e)
    {
        return static_cast<std::underlying_type_t<T>>(e);
    };

    template <typename T>
    std::underlying_type_t<T> compare_enums(T e1, T e2)
    {
        return cast_enum_to_base(e1) & cast_enum_to_base(e2);
    };

    enum class Direction : int8_t
    {
        Clockwise = -1,
        Neutral = 0,
        Counterclockwise = 1
    };

    enum class State : uint8_t
    {
        Undefined = 0,
        Manual = 1,
        Running = 2,
        Accelerating = 6,
        Decelerating = 10,
        Paused = 16,
        Stopped = 32,
        Inhibited = 96,
        EmergencyStop = 224
    };

    // Fast integer square root for 64-bit values (up to 44 bits input range)                                                                                       
    inline uint64_t isqrt64(uint64_t val) {                                                                                                                            
        uint64_t root = 0;                                                                                                                                           
        // For UQ20x12 shifted by 12 bits, max shifted value is 44 bits.                                                                                             
        // 1ULL << 44 is the highest power-of-4 bit position needed.                                                                                                 
        uint64_t bit = 1ULL << 62;                                                                                                                                   
                                                                                                                                                                     
        while (bit > val) {                                                                                                                                            
            bit >>= 2;                                                                                                                                               
        }                                                                                                                                                            
                                                                                                                                                                     
        while (bit != 0) {                                                                                                                                           
            if (val >= root + bit) {                                                                                                                                   
                val -= root + bit;                                                                                                                                     
                root = (root >> 1) + bit;                                                                                                                            
            } else {                                                                                                                                                 
                root >>= 1;                                                                                                                                          
            }                                                                                                                                                        
            bit >>= 2;                                                                                                                                               
        }                                                                                                                                                            
                                                                                                                                                                     
        return root;                                                                                                                          
    }

    // Computes the square root of a UQ20x12 fixed-point value cleanly                                                                                              
    inline UQ20x12 sqrt(UQ20x12 val) {     
        constexpr uint64_t scale = UQ20x12::Scale; // 2^12 = 4096                                                                                                                          
        uint64_t val_raw = static_cast<uint64_t>(val.getInternal());

        if (val_raw == 0) {
            return UQ20x12::fromInternal(0);
        }                                                                                                                                                                                                                     
                                                                                                        
        uint64_t root_raw = isqrt64(val_raw * scale);                                                                                                                        
                                                                                                                   
        return UQ20x12::fromInternal(root_raw);                                                                                                                      
    }
}

#endif // STEPPER_HELPER_HPP