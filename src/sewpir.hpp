#ifndef SEW_PIR_HPP
#define SEW_PIR_HPP
#include <Arduino.h>

class SewPIR {
    public:
        SewPIR(uint8_t pin);
        void init();
        boolean isMotionDetected();
        void registerCallback(void (*callback)(boolean detectedMotion));
        void handlePIR();
    private:
        uint8_t motionPin;
        int motionDetected = LOW;
        void (*callback)(boolean detectedMotion);
};
#endif