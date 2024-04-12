#pragma once

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

void set_value_and_log(uint pin, uint value_usec);
