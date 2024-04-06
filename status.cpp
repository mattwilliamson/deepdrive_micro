#include "status.h"

StatusManager& StatusManager::getInstance() {
    static StatusManager instance;
    return instance;
}

Status StatusManager::get() const {
    return status;
}

void StatusManager::set(Status new_status) {
    // Push the new status to the other core
    // multicore_fifo_push_blocking(static_cast<uint32_t>(new_status));
    status = new_status;
}

StatusManager::StatusManager() : status(Status::Init) {}
