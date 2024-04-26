#ifndef ANALOG_SENSORS_HPP
#define ANALOG_SENSORS_HPP

#include <algorithm>
#include <cstdint>

extern "C" {
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
}

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
  /**
   * @brief Initializes the analog sensors.
   */
  AnalogSensors();

  /**
   * @brief Gets the battery voltage.
   * @return The battery voltage in volts.
   */
  float getBatteryVoltage();

  /**
   * @brief Gets the temperature.
   * @return The temperature in degrees Celsius.
   */
  double getTemperature();

  /**
   * @brief Converts a voltage value to a percentage for a lipo battery.
   * @param voltage The voltage value to convert.
   * @return The percentage value.
   */
  static float convertVoltageToPercentage(float voltage);
};

#endif  // ANALOG_SENSORS_HPP