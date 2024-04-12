#pragma once

#include <cstdint>

extern "C" {
    #include "pico/multicore.h"
}

enum class Status {
    Init,
    Connecting,
    Connected,
    Active,
    Executing,
    Success,
    Warning,
    Error,
    Rebooted
};

class StatusManager {
public:
    static StatusManager& getInstance();

    Status get() const;
    void set(Status new_status);

private:
    StatusManager();
    StatusManager(const StatusManager&) = delete;
    StatusManager& operator=(const StatusManager&) = delete;

    Status status;
};
