#include "sewpir.hpp"

SewPIR::SewPIR(uint8_t pin)  {
    this->motionPin = pin;
}

void SewPIR::init() {
    pinMode(motionPin, INPUT);
}

boolean SewPIR::isMotionDetected() {
  return this->motionDetected == HIGH ? true : false;
}

void SewPIR::registerCallback(void (*callback)(boolean detectedMotion)) {
    this->callback = callback;
}

void SewPIR::handlePIR() {
    int state = digitalRead(motionPin);
    if (state != motionDetected) {
        motionDetected = state;
        this->callback(isMotionDetected());
    }
}