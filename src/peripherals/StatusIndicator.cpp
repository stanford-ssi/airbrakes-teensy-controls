#include "StatusIndicator.h"

#include <Arduino.h>

StatusIndicator::StatusIndicator(int redPin, int greenPin, int bluePin) {
  this->redPin = redPin;
  this->greenPin = greenPin;
  this->bluePin = bluePin;
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  currentColor = OFF;
}

void StatusIndicator::flash(Color color, int durationMs) {
  this->durationMs = durationMs;
  currentColor = color;
  switch (color) {
    case RED:
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, HIGH);
      break;
    case GREEN:
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
      break;
    case BLUE:
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, LOW);
      break;
    case ORANGE:
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
      break;
    case WHITE:
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
      break;
    default:
      break;
  }
  delay(durationMs / 2);
  off();
  delay(durationMs / 2);
}

void StatusIndicator::solid(Color color) {
  currentColor = color;
  switch (color) {
    case RED:
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, HIGH);
      break;
    case GREEN:
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
      break;
    case BLUE:
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, LOW);
      break;
    case ORANGE:
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
      break;
    case WHITE:
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
      break;
    default:
      break;
  }
}

void StatusIndicator::rainbow() {
  // Smooth HSV hue rotation — full cycle every 3 seconds
  float hue = fmodf(millis() / 3000.0f, 1.0f) * 6.0f;
  int sector = (int)hue;
  float frac = hue - sector;

  float r = 0, g = 0, b = 0;
  switch (sector % 6) {
    case 0: r = 1;      g = frac;   b = 0;      break;  // red → yellow
    case 1: r = 1-frac; g = 1;      b = 0;      break;  // yellow → green
    case 2: r = 0;      g = 1;      b = frac;   break;  // green → cyan
    case 3: r = 0;      g = 1-frac; b = 1;      break;  // cyan → blue
    case 4: r = frac;   g = 0;      b = 1;      break;  // blue → magenta
    case 5: r = 1;      g = 0;      b = 1-frac; break;  // magenta → red
  }

  // Active-low: 0 = full on, 255 = off
  analogWrite(redPin,   (int)(255 * (1.0f - r)));
  analogWrite(greenPin, (int)(255 * (1.0f - g)));
  analogWrite(bluePin,  (int)(255 * (1.0f - b)));
}

void StatusIndicator::off() {
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin, HIGH);
  currentColor = OFF;
}