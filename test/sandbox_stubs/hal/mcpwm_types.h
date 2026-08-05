#ifndef HAL_MCPWM_TYPES_H
#define HAL_MCPWM_TYPES_H

#include <cstdint>

typedef void* mcpwm_timer_handle_t;
typedef void* mcpwm_oper_handle_t;
typedef void* mcpwm_cmpr_handle_t;
typedef void* mcpwm_gen_handle_t;

typedef enum {
    MCPWM_TIMER_CLK_SRC_DEFAULT = 0
} mcpwm_timer_clock_source_t;

typedef enum {
    MCPWM_TIMER_COUNT_MODE_UP = 0,
    MCPWM_TIMER_COUNT_MODE_UP_DOWN = 1
} mcpwm_timer_count_mode_t;

typedef enum {
    MCPWM_TIMER_DIRECTION_UP = 0,
    MCPWM_TIMER_DIRECTION_DOWN = 1
} mcpwm_timer_direction_t;

#endif // HAL_MCPWM_TYPES_H
