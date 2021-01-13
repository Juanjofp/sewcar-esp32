#include "sewdistance.hpp"

SewDistance::SewDistance(uint8_t trigger, uint8_t echo, uint16_t maxDistance): pingClient(trigger, echo, maxDistance)  {
    this->triggerPin = trigger;
    this->echoPin = echo;
    this->maxDistance = maxDistance;
}

void SewDistance::init() {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

int SewDistance::getDistance() {
  return pingClient.ping_cm();
}

void SewDistance::registerCallback(uint16_t millis, void (*callback)(int distance)) {
    this->timeBetweenCalls = millis;
    this->callback = callback;
}

void SewDistance::handleDistances() {
    if (millis() -  currentTime > timeBetweenCalls) {
        currentTime = millis();
        this->callback(getDistance());
    }
}