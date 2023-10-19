#include <Arduino.h>

class NonBlockingTimer{
	long lastTime;
	int milliDelay;

    public:
        void setFrequency(double frequency) {
            lastTime = 0;
            milliDelay = round(1000 / frequency);
        }

        void setPeriod(double period){
            lastTime = 0;
            milliDelay = round(1000 * period);
        }

        bool isReady() {
            long currentTime = millis();
            if (currentTime - lastTime >= milliDelay) {
                lastTime = currentTime;
                return true;
            }else {
                return false;
            }
        }
};