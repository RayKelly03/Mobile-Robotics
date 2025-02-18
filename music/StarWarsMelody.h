#ifndef STAR_WARS_MELODY_H
#define STAR_WARS_MELODY_H

#include <Arduino.h>

// Define musical notes (frequencies in Hz)
#define a 440
#define aS 455
#define b 466
#define cH 523
#define cSH 554
#define dH 587
#define dSH 622
#define eH 659
#define fH 698
#define fSH 740
#define gH 784
#define gSH 830
#define aH 880
#define f 349
#define gS 415

// Pin assignments
const int buzzerPin = 17;
const int ledPin1 = 12;
const int ledPin2 = 13;

// Counter for LED blinking
int counter = 0;

// Function prototypes
void playStarWarsMelody();
void firstSection();
void secondSection();
void beep(int note, int duration);

// Function to play the full melody
void playStarWarsMelody() {
    firstSection();
    secondSection();

    // Variant 1
    beep(f, 250);
    beep(gS, 500);
    beep(f, 350);
    beep(a, 125);
    beep(cH, 500);
    beep(a, 375);
    beep(cH, 125);
    beep(eH, 650);
    delay(500);

    secondSection();

    // Variant 2
    beep(f, 250);
    beep(gS, 500);
    beep(f, 375);
    beep(cH, 125);
    beep(a, 500);
    beep(f, 375);
    beep(cH, 125);
    beep(a, 650);

    delay(650);
}

// Function to play a beep with a note and duration
void beep(int note, int duration) {
    tone(buzzerPin, note, duration);

    if (counter % 2 == 0) {
        digitalWrite(ledPin1, HIGH);
        delay(duration);
        digitalWrite(ledPin1, LOW);
    } else {
        digitalWrite(ledPin2, HIGH);
        delay(duration);
        digitalWrite(ledPin2, LOW);
    }

    noTone(buzzerPin);
    delay(50);
    counter++;
}

// First section of the melody
void firstSection() {
    beep(a, 500);
    beep(a, 500);
    beep(a, 500);
    beep(f, 350);
    beep(cH, 150);
    beep(a, 500);
    beep(f, 350);
    beep(cH, 150);
    beep(a, 650);

    delay(500);

    beep(eH, 500);
    beep(eH, 500);
    beep(eH, 500);
    beep(fH, 350);
    beep(cH, 150);
    beep(gS, 500);
    beep(f, 350);
    beep(cH, 150);
    beep(a, 650);

    delay(500);
}

// Second section of the melody
void secondSection() {
    beep(aH, 500);
    beep(a, 300);
    beep(a, 150);
    beep(aH, 500);
    beep(gSH, 325);
    beep(gH, 175);
    beep(fSH, 125);
    beep(fH, 125);
    beep(fSH, 250);

    delay(325);

    beep(aS, 250);
    beep(dSH, 500);
    beep(dH, 325);
    beep(cSH, 175);
    beep(cH, 125);
    beep(b, 125);
    beep(cH, 250);

    delay(350);
}

#endif // STAR_WARS_MELODY_H
