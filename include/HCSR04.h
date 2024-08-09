#ifndef HC_SR04_H
#define HC_SR04_H

#include <Arduino.h>

/**
 * @brief Speed of sound in meters per microsecond.
 */
#define SPEED_OF_SOUND_M_PER_US 0.000343

/**
 * @brief Minimum range of the sensor in meters (2 cm).
 */
#define MIN_RANGE_METERS 0.02

/**
 * @brief Maximum range of the sensor in meters (12 m).
 */
#define MAX_RANGE_METERS 12.0

/**
 * @brief Factor to convert time to distance in meters.
 * 
 * The factor accounts for the round trip of the sound wave.
 */
#define TIME_TO_DISTANCE_FACTOR (SPEED_OF_SOUND_M_PER_US / 2.0)

/**
 * @brief The duration for which the trigger pin is held low before sending a high pulse.
 * 
 * A short pulse of 2 microseconds is required before sending the high pulse to trigger the sensor.
 */
#define TRIGGER_PULSE_LOW_TIME 2

/**
 * @brief The duration for which the trigger pin is held high to start the sensor measurement.
 * 
 * A pulse of 10 microseconds is recommended by the HC-SR04 datasheet to trigger the sensor.
 */
#define TRIGGER_PULSE_HIGH_TIME 10

/**
 * @class HCSR04
 * @brief A class to interface with the HC-SR04 ultrasonic distance sensor.
 *
 * This class provides methods to trigger the sensor, handle the echo response via an interrupt,
 * filter outliers, and calculate the average distance measured over a buffer of values.
 *
 * Example usage:
 * @code
 * #include "HCSR04.h"
 *
 * // Create instances of HCSR04
 * HCSR04 sensor1(7, 6);  // Trig pin: 7, Echo pin: 6
 * HCSR04 sensor2(8, 9);  // Trig pin: 8, Echo pin: 9
 *
 * void sensor1ISR() {
 *     sensor1.echoISR();
 * }
 *
 * void sensor2ISR() {
 *     sensor2.echoISR();
 * }
 *
 * void setup() {
 *     Serial.begin(115200);
 *
 *     // Initialize sensors
 *     sensor1.begin();
 *     sensor2.begin();
 *
 *     // Attach interrupts to the echo pins
 *     attachInterrupt(digitalPinToInterrupt(6), sensor1ISR, CHANGE);
 *     attachInterrupt(digitalPinToInterrupt(9), sensor2ISR, CHANGE);
 * }
 *
 * void loop() {
 *     // Update the sensors
 *     sensor1.update();
 *     sensor2.update();
 *
 *     // Get distances
 *     float distance1 = sensor1.getDistance();
 *     float distance2 = sensor2.getDistance();
 *
 *     // Print distances
 *     Serial.print("Distance 1: ");
 *     Serial.println(distance1);
 *     Serial.print("Distance 2: ");
 *     Serial.println(distance2);
 *
 *     delay(500);
 * }
 * @endcode
 */
class HCSR04 {
public:
    /**
     * @brief Constructor for the HCSR04 class.
     * 
     * @param trigPin The Arduino pin connected to the trigger pin of the sensor.
     * @param echoPin The Arduino pin connected to the echo pin of the sensor.
     * @param bufferSize The number of readings to keep in the buffer for averaging.
     */
    HCSR04(uint8_t trigPin, uint8_t echoPin, uint8_t bufferSize = 10);

    /**
     * @brief Initializes the sensor pins.
     */
    void begin();

    /**
     * @brief Triggers the sensor and starts a new measurement.
     */
    void update();

    /**
     * @brief Interrupt Service Routine (ISR) to handle the echo signal.
     */
    void echoISR();

    /**
     * @brief Calculates and returns the average distance from the buffer.
     * 
     * @return The average distance in meters. Returns infinity if all distances exceed the max range.
     */
    float getDistance();

private:
    /**
     * @brief Adds a new distance measurement to the buffer.
     * 
     * @param value The distance value to be added to the buffer.
     */
    void addToBuffer(float value);

    /**
     * @brief Calculates the average distance from the buffer.
     * 
     * @return The average distance in meters. Returns infinity if all distances exceed the max range.
     */
    float calculateAverage();

    uint8_t trigPin;       ///< The trigger pin for the sensor.
    uint8_t echoPin;       ///< The echo pin for the sensor.
    uint8_t bufferSize;    ///< The size of the buffer to store recent distance measurements.
    volatile unsigned long startTime; ///< The start time of the pulse in microseconds.
    volatile unsigned long endTime;   ///< The end time of the pulse in microseconds.
    volatile bool measuring;          ///< Flag indicating if a measurement is in progress.
    float *distanceBuffer;            ///< Pointer to the buffer storing recent distance measurements.
    uint8_t bufferIndex;              ///< Index to the current position in the buffer.
    bool bufferFull;                  ///< Flag indicating if the buffer is full.
};

#endif
