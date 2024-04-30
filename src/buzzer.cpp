#include "buzzer.hpp"


// Initialize static constants
note_t Buzzer::positiveMelody[] = {
    {NOTE_C4, 16},
    {NOTE_C5, 32},
    {NOTE_C6, 64},
    {REST, 8},
    {MELODY_END, 0}
};

note_t Buzzer::negativeMelody[] = {
    {NOTE_C6, 16},
    {NOTE_C5, 32},
    {NOTE_C4, 64},
    {REST, 8},
    {MELODY_END, 0}
};

note_t Buzzer::warningMelody[] = {
    {NOTE_C4, 16},
    {NOTE_C4, 16},
    {NOTE_C4, 16},
    {REST, 8},
    {MELODY_END, 0}
};

note_t Buzzer::errorMelody[] = {
    {NOTE_C6, 16},
    {NOTE_C6, 16},
    {NOTE_C6, 16},
    {REST, 8},
    {MELODY_END, 0}
};;

Buzzer::Buzzer(int pin) {
    tone_init(&generator_, pin);

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
        case Tune::POSITIVE:
            playMelody(positiveMelody, 1);
            break;
        case Tune::NEGATIVE:
            playMelody(negativeMelody, 1);
            break;
        case Tune::WARNING:
            playMelody(warningMelody, 1);
            break;
        case Tune::ERROR:
            playMelody(errorMelody, 1);
            break;
        default:
            break;
    }
}

void Buzzer::stop() {
    stop_melody(&generator_);
    playing_ = false;
}