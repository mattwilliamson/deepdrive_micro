#include "buzzer.hpp"

// Initialize static constants
note_t Buzzer::startupMelody[] = {
    {NOTE_A6, 16},
    {NOTE_C7, 16},
    // {NOTE_E7, 8},
    // {NOTE_E8, 8},
    // {REST, 8},
    {MELODY_END, 0},
};

note_t Buzzer::positiveMelody[] = {
    {NOTE_C4, 16},
    {NOTE_C5, 32},
    {NOTE_C6, 64},
    {REST, 8},
    {MELODY_END, 0}};

note_t Buzzer::negativeMelody[] = {
    {NOTE_C6, 16},
    {NOTE_C5, 32},
    {NOTE_C4, 64},
    // {REST, 8},
    {MELODY_END, 0}};

note_t Buzzer::warningMelody[] = {
    {NOTE_C4, 16},
    {NOTE_C4, 16},
    {NOTE_C4, 16},
    {REST, 8},
    {MELODY_END, 0}};

note_t Buzzer::errorMelody[] = {
    {NOTE_C6, 16},
    {NOTE_C6, 16},
    {NOTE_C6, 16},
    {REST, 8},
    {MELODY_END, 0}};

note_t Buzzer::rebootedMelody[] = {
    {NOTE_C7, 16},
    {NOTE_A6, 16},
    {NOTE_C5, 16},
    {REST, 16},
    {MELODY_END, 0},
};

Buzzer::Buzzer(int pin) {
#ifdef BUZZER_ENABLED
  tone_init(&generator_, pin);
#endif

  enabled_ = true;
  warning_ = false;
  error_ = false;
  playing_ = false;
  warningPlayed_ = false;
  errorPlayed_ = false;
}

void Buzzer::playMelody(note_t melody_notes[], int repeat) {
  melody(&generator_, melody_notes, repeat);
  playing_ = true;
}
#include "buzzer.hpp"

void Buzzer::playTune(Tune tune) {
  switch (tune) {
    case Tune::STARTUP:
      //   playTone(NOTE_A6, 200);
      //   sleep_ms(202);
      //   playTone(NOTE_C7, 200);
      //   sleep_ms(202);
      //   playTone(NOTE_E8, 200);
      //   sleep_ms(202);
      break;
    case Tune::POSITIVE:
      //   playTone(NOTE_C7, 200);
      //   sleep_ms(200);
      break;
    case Tune::NEGATIVE:
      //   playTone(NOTE_C5, 200);
      //   sleep_ms(200);
      //   playTone(NOTE_C5, 200);
      //   sleep_ms(202);
      break;
    case Tune::WARNING:
      //   playTone(NOTE_C5, 200);
      //   sleep_ms(200);
      //   playTone(NOTE_C5, 200);
      //   sleep_ms(200);
      //   playTone(NOTE_C5, 200);
      //   sleep_ms(200);
      //   playTone(NOTE_C5, 200);
      //   sleep_ms(202);
      break;
    case Tune::ERROR:
      //   playTone(NOTE_C5, 400);
      //   sleep_ms(400);
      //   playTone(NOTE_C5, 400);
      //   sleep_ms(400);
      //   playTone(NOTE_C5, 400);
      //   sleep_ms(400);
      playTone(NOTE_C5, 200);
      sleep_ms(202);
      break;
    case Tune::REBOOTED:
      // playMelody(rebootedMelody, 0);
      //   playTone(NOTE_C5, 200);
      //   sleep_ms(202);
      //   playTone(NOTE_C5, 300);
      //   sleep_ms(302);
      break;
    default:
      break;
  }
}

void Buzzer::playTone(int frequency, int duration) {
#ifdef BUZZER_ENABLED
  tone(&generator_, frequency, duration);
  playing_ = true;
#endif
}

void Buzzer::stop() {
#ifdef BUZZER_ENABLED
  stop_tone(&generator_);
  stop_melody(&generator_);
  playing_ = false;
#endif
}