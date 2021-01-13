#ifndef SEW_MOTORS
#define SEW_MOTORS

#include <Arduino.h>

class SewMotors {
    public:
        SewMotors(uint8_t enabled, uint8_t forward, uint8_t backward);
        void init();
        void runMotor(uint8_t started, uint8_t reverse, uint8_t power);
    private:
        uint8_t enabled;
        uint8_t forward;
        uint8_t backward;
};
#endif