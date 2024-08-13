#include "StatusLED.h"

StatusLED::StatusLED(int numLEDs, int pin) : strip(numLEDs, pin, NEO_GRB + NEO_KHZ800) {
    isTransition = false;
    previousMillis = 0;
    blinkInterval = 1000; // default to 1 second blink interval
    blinkCount = 0;
    ledPin = pin;
}

void StatusLED::begin() {
    strip.begin();
    strip.clear();
}

void StatusLED::setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
    for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(r * brightness / 255, g * brightness / 255, b * brightness / 255));
    }
    strip.show();
}

void StatusLED::update() {
    Status currentStatus = StatusManager::getInstance().getStatus();
    unsigned long currentMillis = millis();
    if (isTransition) {
        // Handle transition from CONNECTING to CONNECTED
        if (blinkCount < 6) { // Blink green twice (each blink consists of two states: on and off)
            if (currentMillis - previousMillis >= 250) { // 250ms blink interval for transition
                previousMillis = currentMillis;
                if (blinkCount % 2 == 0) {
                    setColor(0, 255, 0); // Green on
                } else {
                    setColor(0, 0, 0);   // Off
                }
                blinkCount++;
            }
        } else {
            setColor(255, 255, 255, 128); // White at 50% brightness
            isTransition = false;
        }
    } else {
        switch (currentStatus) {
            case ERROR:
                if (currentMillis - previousMillis >= 500) { // Blink red every 500ms
                    previousMillis = currentMillis;
                    if (strip.getPixelColor(0) == strip.Color(255, 0, 0)) {
                        setColor(0, 0, 0); // Off
                    } else {
                        setColor(255, 0, 0); // Red
                    }
                }
                break;

            case CONNECTING:
                if (currentMillis - previousMillis >= 1000) { // Slow blink every 1 second
                    previousMillis = currentMillis;
                    if (strip.getPixelColor(0) == strip.Color(0, 0, 255)) {
                        setColor(0, 0, 0); // Off
                    } else {
                        setColor(0, 0, 255); // Blue for connecting (or you can change this to any other color)
                    }
                }
                break;

            case CONNECTED:
                setColor(255, 255, 255, 128); // White at 50% brightness
                break;
        }
    }
}
