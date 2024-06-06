#ifndef ANALOG_SENSORS_HPP
#define ANALOG_SENSORS_HPP

#include <algorithm>
#include <cstdint>

extern "C" {
#include "hardware/adc.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
}

#include "battery.hpp"
#include "config.h"

/**
 * @brief Class representing analog sensors.
 */
class AnalogSensors {
 private:
  // How many samples to take and average for the analog sensors
  static const int NUM_SAMPLES = 10;

  mutex_t lock_;

 public:
  LiPoBattery battery;

  /**
   * @brief Initializes the analog sensors.
   */
  AnalogSensors();

  /**
   * @brief Reads the battery voltage and updates internal battery state.
   */
  void updateBatteryVoltage();

  /**
   * @brief Gets the temperature.
   * @return The temperature in degrees Celsius.
   */
  double getTemperature();

  /**
   * @brief Gets the battery object.
   * @return The battery object.
   */
  LiPoBattery& getBattery() {
    return battery;
  }
};

#endif  // ANALOG_SENSORS_HPP