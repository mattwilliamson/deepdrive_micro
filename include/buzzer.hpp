#ifndef BUZZER_HPP
#define BUZZER_HPP

extern "C" {
#include "pico/multicore.h"
#include "pwm-tone/pwm-tone.h"
}

#include "config.h"

class Buzzer {
 private:
  tonegenerator_t generator_;
  float batteryVoltage_;
  bool warning_;
  bool error_;
  bool playing_;
  bool enabled_;
  bool warningPlayed_;
  bool errorPlayed_;

 public:
  enum class Tune {
    STARTUP,
    POSITIVE,
    NEGATIVE,
    WARNING,
    ERROR,
    REBOOTED
  };

  Buzzer(int pin);
  void playMelody(note_t melody[], int repeat);
  void playTune(Tune tune);
  void playTone(int frequency, int duration);
  void stop();
  void update();

  // Define melodies as static constants
  static note_t startupMelody[];
  static note_t positiveMelody[];
  static note_t negativeMelody[];
  static note_t warningMelody[];
  static note_t errorMelody[];
  static note_t rebootedMelody[];
};

#endif