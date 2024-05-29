#ifndef LED_RING_STATUS_HPP
#define LED_RING_STATUS_HPP

#include "led_ring.hpp"

/**
 * @class LEDRingStatus
 * @brief Singleton class representing the status of an LED ring.
 * 
 * The LEDRingStatus class provides access to a single instance of an LED ring status.
 * It follows the Singleton design pattern to ensure that only one instance of the class exists.
 * The class provides a method to retrieve the LED ring object.
 */
class LEDRingStatus {
public:
    /**
     * @brief Get the instance of the LEDRingStatus class.
     * @return The instance of the LEDRingStatus class.
     */
    static LEDRingStatus& getInstance() {
        static LEDRingStatus instance;
        return instance;
    }

    LEDRingStatus(const LEDRingStatus&) = delete;
    LEDRingStatus& operator=(const LEDRingStatus&) = delete;

    /**
     * @brief Get the LED ring object.
     * @return The LED ring object.
     */
    LEDRing& getLEDRing() {
        return *global_led_ring;
    }

private:
    LEDRing* global_led_ring;

    /**
     * @brief Private constructor to prevent instantiation of the class.
     * 
     * The constructor initializes the global_led_ring object with the specified parameters.
     */
    LEDRingStatus() : global_led_ring(new LEDRing(LED_RING_PIN, LED_RING_PIO, LED_RING_NUM_PIXELS)) {}

    /**
     * @brief Destructor to clean up the global_led_ring object.
     */
    ~LEDRingStatus() {
        delete global_led_ring;
    }
};;



#endif // LED_RING_STATUS_HPP