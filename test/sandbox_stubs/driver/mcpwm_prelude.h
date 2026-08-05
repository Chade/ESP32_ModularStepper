#ifndef DRIVER_MCPWM_PRELUDE_H
#define DRIVER_MCPWM_PRELUDE_H

#include "mcpwm_types.h"
#include "mcpwm_timer.h"
#include "mcpwm_gen.h"
#include "mcpwm_sync.h"

struct mcpwm_operator_config_t {
    int group_id;
    int intr_priority;
    struct {
        uint32_t update_gen_action_on_tez : 1;
        uint32_t update_gen_action_on_tep : 1;
        uint32_t update_gen_action_on_sync : 1;
        uint32_t update_dead_time_on_tez : 1;
        uint32_t update_dead_time_on_tep : 1;
        uint32_t update_dead_time_on_sync : 1;
    } flags;
};

struct mcpwm_comparator_config_t {};

struct mcpwm_compare_event_data_t {
    mcpwm_timer_direction_t direction;
    uint32_t compare_ticks;
};

inline int mcpwm_new_operator(const mcpwm_operator_config_t*, mcpwm_oper_handle_t*) { return 0; }
inline int mcpwm_operator_connect_timer(mcpwm_oper_handle_t, mcpwm_timer_handle_t) { return 0; }

#endif // DRIVER_MCPWM_PRELUDE_H
