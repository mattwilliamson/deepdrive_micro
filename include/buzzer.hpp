#ifndef BUZZER_HPP
#define BUZZER_HPP

extern "C" {
#include "pwm-tone/pwm-tone.h"
#include "pico/multicore.h"
}

#include "config.h"
#include "analog_sensors.hpp"

class Buzzer {
    private:
        tonegenerator_t generator_;
        // AnalogSensors analogSensors_;
        float batteryVoltage_;
        bool warning_;
        bool error_;
        bool playing_;
        bool enabled_;
        bool warningPlayed_;
        bool errorPlayed_;
        
    public:
        Buzzer(int pin);
        void playMelody(note_t melody[], int repeat);
        void playPositive();
        void playNegative();
        void playWarning();
        void playError();
        void stop();
        void update();

        // Define melodies as static constants
        static note_t positiveMelody[];
        static note_t negativeMelody[];
        static note_t warningMelody[];
        static note_t errorMelody[];

        // Define tune enum
        enum class Tune {
            POSITIVE,
            NEGATIVE,
            WARNING,
            ERROR
        };

        // Add playTune function
        void playTune(Tune tune);
};

#endif