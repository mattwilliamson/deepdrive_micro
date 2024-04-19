#ifndef DIAGNOSTIC_H
#define DIAGNOSTIC_H

extern "C" {
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <rcl/rcl.h>
}

namespace Diagnostics {
    const int IMU = 0;
    const int CORE_0 = 1;
    const int CORE_1 = 2;
}



#endif  // DIAGNOSTIC_H