/**
 * @file status.hpp
 * @brief Defines the Status enum and StatusManager class for managing the status of a system.
 */

#ifndef STATUS_HPP
#define STATUS_HPP

#include <cstdint>

extern "C" {
#include "pico/multicore.h"
}

/**
 * @enum Status
 * @brief Represents the possible statuses of a system.
 */
enum class Status {
    Init,         /**< Initialization status */
    Connecting,   /**< Connecting status */
    Connected,    /**< Connected status */
    Active,       /**< Active status */
    Executing,    /**< Executing status */
    Success,      /**< Success status */
    Warning,      /**< Warning status */
    Error,        /**< Error status */
    Rebooted      /**< Rebooted status */
};

/**
 * @class StatusManager
 * @brief Singleton class for managing the status of a system.
 */
class StatusManager {
 public:
    /**
     * @brief The StatusManager class is responsible for managing the status of the rover.
     *
     * The StatusManager class provides a singleton instance that can be accessed using the
     * `getInstance()` method. It allows other parts of the code to retrieve and update the
     * status of the rover.
     */
    static StatusManager& getInstance();

    /**
     * @brief Gets the current status.
     * @return The current status.
     */
    Status get() const;

    /**
     * @brief Sets a new status.
     * @param new_status The new status to set.
     */
    void set(Status new_status);

    /**
     * @brief Default constructor.
     */
    StatusManager();

 private:
    StatusManager(const StatusManager&) = delete;
    StatusManager& operator=(const StatusManager&) = delete;

    Status status; /**< The current status. */
};

#endif  // STATUS_HPP