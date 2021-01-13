#ifndef SEW_DISTANCE
#define SEW_DISTANCE

#include <NewPing.h>

class SewDistance {
    public:
        SewDistance(uint8_t trigger, uint8_t echo, uint16_t maxDistance = MAX_SENSOR_DISTANCE);
        void init();
        int getDistance();
        void registerCallback(uint16_t millis, void (*callback)(int distance));
        void handleDistances();
    private:
        NewPing pingClient;
        uint8_t triggerPin;
        uint8_t echoPin;
        unsigned int maxDistance;
        unsigned long timeBetweenCalls = 1000;
        unsigned long currentTime = 0;
        void (*callback)(int distance);
};
#endif