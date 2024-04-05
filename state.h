#pragma once

#include "pico/multicore.h"

typedef enum {
    STATE_INIT,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_ACTIVE,
    STATE_EXECUTING,
    STATE_SUCCESS,
    STATE_ERROR
} state_t;


state_t get_state();

void set_state(state_t new_state);