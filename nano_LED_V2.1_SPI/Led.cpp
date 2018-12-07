#include "Arduino.h"
#include "Led.h"

void Led::Breathe()
{

}

void Led::Transition()
{
   if ((endTime - startTime) <= 0)
    {
      analogWrite(pin, 255 - maxPower);
    }
  if (!done)
  {
    if (endTime > millis())
    {
      int power;
      if (maxPower > lastPower)
        power = lastPower + ((maxPower-lastPower) * (1.0 - ((float)(endTime - millis()) / (float)(endTime - startTime)))); // get the relative time of the animation
      else
        power = maxPower + ((lastPower-maxPower) * ((float)(endTime - millis()) / (float)(endTime - startTime))); // get the relative time of the animation
        analogWrite(pin, 255 - power);
      }
    else
    {
      isTransition = false;
      done = true;
      endTime = 0;
    }
  }
}

void Led::SetDelay(unsigned long duration)
{
  if (duration > 0)
  {
    if (!isTransition && !done)
    {
      isTransition = true;
      startTime = millis();
      endTime = millis() + duration;
    }
  } else {
    startTime = 0;
    endTime = 0;
  }
}

void Led::SetMaxPower(int power)
{
  if (power != maxPower)
  {
    lastPower = maxPower;
    maxPower = power;
    done = false;
  }
}

