#ifndef STATUS_MANAGER_H
#define STATUS_MANAGER_H

enum Status {
    ERROR,
    CONNECTING,
    CONNECTED
};

class StatusManager {
private:
    Status currentStatus;

    // Private constructor to prevent instantiation
    StatusManager() : currentStatus(ERROR) {}

public:
    // Delete copy constructor and assignment operator to enforce singleton pattern
    StatusManager(const StatusManager&) = delete;
    void operator=(const StatusManager&) = delete;

    static StatusManager& getInstance() {
        static StatusManager instance;
        return instance;
    }

    void setStatus(Status newStatus) {
        currentStatus = newStatus;
    }

    Status getStatus() const {
        return currentStatus;
    }
};

#endif // STATUS_MANAGER_H
