#include "state.h"

static state_t state = STATE_INIT;

state_t get_state() {
    return state;
}

void set_state(state_t new_state) {
    state = new_state;
    multicore_fifo_push_blocking(new_state);
}