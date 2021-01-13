#include <Arduino.h>
#include <analogWrite.h>
#include "sewmotors.hpp"

SewMotors::SewMotors(uint8_t enabled, uint8_t forward, uint8_t backward) {
    this->enabled = enabled;
    this->forward = forward;
    this->backward = backward;
}

void SewMotors::init() {
    pinMode(enabled, OUTPUT);
    pinMode(forward, OUTPUT);
    pinMode(backward, OUTPUT);
}

void SewMotors::runMotor(uint8_t started, uint8_t reverse, uint8_t power) {
    if (started == 0) {
        digitalWrite(forward, LOW);
        digitalWrite(backward, LOW);
        analogWrite(enabled, 0);
    }
    else {
        if(reverse == 0) {
            digitalWrite(forward, HIGH);
            digitalWrite(backward, LOW);
            analogWrite(enabled, power);
        }
        else {
            digitalWrite(forward, LOW);
            digitalWrite(backward, HIGH);
            analogWrite(enabled, power);
        }
    }
}