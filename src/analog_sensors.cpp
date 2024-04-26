#include "analog_sensors.hpp"

AnalogSensors::AnalogSensors() {
  mutex_init(&lock_);
  adc_init();
  adc_gpio_init(PIN_BATTERY_VOLTAGE);
  adc_set_temp_sensor_enabled(true);

  // set GPIO23 to output to disable power saving mode
  gpio_init(23);
}

float AnalogSensors::getBatteryVoltage() {
  // TODO: Average battery over time
  

  // Disable power saving
  gpio_put(23, 1);

  adc_select_input(PIN_BATTERY_VOLTAGE_INPUT);
  double sum = 0.0;
  for (int i = 0; i < NUM_SAMPLES;
       i++) {  // Use the class constant for the loop condition
    uint16_t rawValue = adc_read();
    sum += rawValue;
  }
  double average_adc = sum / NUM_SAMPLES;  // Use the class constant for
                                           // calculating the average voltage
  const float conversion_factor = BATTERY_VOLTAGE_REFERENCE / (1 << 12);
  float volts = conversion_factor * average_adc;
  // ADC raw value = 2356, multimeter raw value = 11.08
  // ADC_REF = 3.277, Voltage Divider = 1.902, ADC Calculated
  // =1.8839550018310547

  // Convert from ADC voltage to battery voltage
  float battery_volts = volts * BATTERY_VOLTAGE_CONVERSION;

  // Enable power saving
  // gpio_put(23, 0);

  return battery_volts;
}

double AnalogSensors::getTemperature() {
  mutex_enter_blocking(&lock_);

  // Disable power saving
  gpio_put(23, 1);

  adc_select_input(4);

  /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
  const double conversionFactor = 3.3 / (1 << 12);

  double adc = (float)adc_read() * conversionFactor;
  double tempC = 27.0 - (adc - 0.706) / 0.001721f;

  // Enable power saving
  gpio_put(23, 0);

  mutex_exit(&lock_);

  return tempC;
}

float AnalogSensors::convertVoltageToPercentage(float voltage) {
  // Assuming a LiPo battery with a voltage range of 3.0V to 4.2V
  // float minVoltage = 3.0 * BATTERY_CELLS;
  // float maxVoltage = 4.2 * BATTERY_CELLS;
  // Base these numbers off discharge graphs
  float minVoltage = 3.3 * BATTERY_CELLS;
  float maxVoltage = 4.0 * BATTERY_CELLS;

  // Calculate the percentage based on the voltage range
  float percentage = (voltage - minVoltage) / (maxVoltage - minVoltage) * 100.0;

  // Ensure the percentage is within the valid range of 0% to 100%
  percentage = std::clamp(percentage, 0.0f, 100.0f);

  return percentage;
}