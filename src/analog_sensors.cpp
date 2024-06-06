#include "analog_sensors.hpp"

AnalogSensors::AnalogSensors() {
  mutex_init(&lock_);
  adc_init();
  adc_gpio_init(PIN_BATTERY_VOLTAGE);
  adc_set_temp_sensor_enabled(true);

  // set GPIO23 to output to disable power saving mode
  gpio_init(23);
}

void AnalogSensors::updateBatteryVoltage() {
  // TODO: Average battery over time
  

  // Disable power saving
  gpio_put(23, 1);

  adc_select_input(PIN_BATTERY_VOLTAGE_INPUT);
  double sum = 0.0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    uint16_t rawValue = adc_read();
    sum += rawValue;
  }
  double average_adc = sum / NUM_SAMPLES;
  const float conversion_factor = BATTERY_VOLTAGE_REFERENCE / (1 << 12);
  float volts = conversion_factor * average_adc;
  // ADC raw value = 2356, multimeter raw value = 11.08
  // ADC_REF = 3.277, Voltage Divider = 1.902, ADC Calculated
  // =1.8839550018310547

  // Convert from ADC voltage to battery voltage
  float battery_volts = volts * BATTERY_VOLTAGE_CONVERSION;

  // Enable power saving
  // gpio_put(23, 0);

  battery.updateVoltage(battery_volts);
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
