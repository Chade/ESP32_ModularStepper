#ifndef DRIVER_MCPWM_TIMER_H
#define DRIVER_MCPWM_TIMER_H

#include "hal/mcpwm_types.h"

struct mcpwm_timer_config_t {
    int group_id;
    mcpwm_timer_clock_source_t clk_src;
    uint32_t resolution_hz;
    mcpwm_timer_count_mode_t count_mode;
    uint32_t period_ticks;
    int intr_priority;
    struct {
        uint32_t update_period_on_empty : 1;
        uint32_t update_period_on_sync : 1;
        uint32_t allow_pd : 1;
    } flags;
};

struct mcpwm_timer_event_data_t {
    mcpwm_timer_direction_t direction;
};

typedef bool (*mcpwm_timer_event_cb_t)(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t* edata, void* user_ctx);

struct mcpwm_timer_event_callbacks_t {
    mcpwm_timer_event_cb_t on_full;
    mcpwm_timer_event_cb_t on_empty;
    mcpwm_timer_event_cb_t on_stop;
};

inline int mcpwm_new_timer(const mcpwm_timer_config_t*, mcpwm_timer_handle_t*) { return 0; }
inline int mcpwm_timer_set_period(mcpwm_timer_handle_t, uint32_t) { return 0; }
inline int mcpwm_timer_register_event_callbacks(mcpwm_timer_handle_t, const mcpwm_timer_event_callbacks_t*, void*) { return 0; }

#endif // DRIVER_MCPWM_TIMER_H
