#include "HCSR04.h"

/**
 * @brief Constructs an HCSR04 object.
 * 
 * Initializes the class with the specified trigger and echo pins, and the buffer size.
 * 
 * @param trigPin The Arduino pin connected to the trigger pin of the sensor.
 * @param echoPin The Arduino pin connected to the echo pin of the sensor.
 * @param bufferSize The number of readings to keep in the buffer for averaging.
 */
HCSR04::HCSR04(uint8_t trigPin, uint8_t echoPin, uint8_t bufferSize)
    : trigPin(trigPin), echoPin(echoPin), bufferSize(bufferSize), measuring(false), bufferIndex(0), bufferFull(false) {
    distanceBuffer = new float[bufferSize];
}

/**
 * @brief Initializes the sensor pins.
 * 
 * Configures the trigger pin as an output and the echo pin as an input.
 */
void HCSR04::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

/**
 * @brief Triggers the sensor and starts a new measurement.
 * 
 * Sends a short pulse to the trigger pin, which starts the measurement.
 */
void HCSR04::update() {
    if (!measuring) {
        // Trigger the sensor
        digitalWrite(trigPin, LOW);
        delayMicroseconds(TRIGGER_PULSE_LOW_TIME);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(TRIGGER_PULSE_HIGH_TIME);
        digitalWrite(trigPin, LOW);
        measuring = true;
    }
}

/**
 * @brief Interrupt Service Routine (ISR) to handle the echo signal.
 * 
 * Measures the duration of the pulse received on the echo pin and calculates the corresponding distance.
 */
void HCSR04::echoISR() {
    if (digitalRead(echoPin) == HIGH) {
        startTime = micros();
    } else {
        endTime = micros();
        unsigned long duration = endTime - startTime;
        float distance = duration * TIME_TO_DISTANCE_FACTOR;  // Convert time to distance in meters

        addToBuffer(distance);
        measuring = false;
    }
}

/**
 * @brief Adds a new distance measurement to the buffer.
 * 
 * Filters out values that are outside the valid range, and updates the buffer.
 * 
 * @param value The distance value to be added to the buffer.
 */
void HCSR04::addToBuffer(float value) {
    if (value < MIN_RANGE_METERS || value > MAX_RANGE_METERS) {  // Filter out outliers
        return;
    }
    distanceBuffer[bufferIndex] = value;
    bufferIndex = (bufferIndex + 1) % bufferSize;
    if (bufferIndex == 0) {
        bufferFull = true;
    }
}

/**
 * @brief Calculates the average distance from the buffer.
 * 
 * Computes the average of the valid distance values stored in the buffer. 
 * If all values exceed the maximum range, returns infinity.
 * 
 * @return The average distance in meters.
 */
float HCSR04::calculateAverage() {
    if (!bufferFull && bufferIndex == 0) {
        return NAN; // No valid data
    }

    float sum = 0;
    uint8_t count = bufferFull ? bufferSize : bufferIndex;

    for (uint8_t i = 0; i < count; i++) {
        sum += distanceBuffer[i];
    }

    float average = sum / count;

    // If all distances are above the max range, return infinity
    if (average > MAX_RANGE_METERS) {
        return INFINITY;
    }

    return average;
}

/**
 * @brief Returns the average distance measured by the sensor.
 * 
 * Calls the calculateAverage function to get the filtered and averaged distance.
 * 
 * @return The average distance in meters.
 */
float HCSR04::getDistance() {
    return calculateAverage();
}
