//input.h - used to map input from the DS4 to data for use in the main .ino

#ifndef input_h
#define input_h

#include "Arduino.h"

class Input {
  public:
    Input();
    int GetVertiIntensity();
    int GetHorizIntensity();
    char GetVertiDirection();
    char GetHorizDirection();
    void LoopCode();

  private:
    USB Usb;
    PS4USB PS4(&Usb);

    bool printAngle, printTouch;
    uint8_t oldL2Value, oldR2Value;
}





#endif
