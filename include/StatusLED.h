#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Adafruit_NeoPixel.h>
#include "StatusManager.h"

class StatusLED {
private:
    Adafruit_NeoPixel strip;
    bool isTransition;
    unsigned long previousMillis;
    int blinkInterval;
    int blinkCount;
    int ledPin;

    void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 255);

public:
    StatusLED(int numLEDs, int pin);

    void begin();
    void update();
};

#endif // STATUS_LED_H
