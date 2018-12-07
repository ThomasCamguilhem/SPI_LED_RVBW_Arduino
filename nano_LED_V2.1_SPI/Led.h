#ifndef LED_H
#define LED_H

class Led {
  public:
    int pin;
    long long startTime, endTime, period;
    int maxPower=0, lastPower = 0;
    bool isTransition = false, done = false;
    void Breathe();
    void Transition();
    void SetDelay(unsigned long duration);
    void SetMaxPower(int power);
};

#endif

