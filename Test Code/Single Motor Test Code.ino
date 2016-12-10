/*
 Robert Gordon's College ROV Dualshock 4 Control Code

 Created 22 November 2016
 */

const float deadZone = 0.10; // range to which no input is read
const int controlPin1 = 2; // connected to pin 7 on the H-bridge
const int controlPin2 = 3; // connected to pin 2 on the H-bridge
const int enablePin = 9;   // connected to pin 1 on the H-bridge
const int directionSwitchPin = 4;  // connected to the switch for direction
const int onOffSwitchStateSwitchPin = 5; // connected to the switch for turning the motor on and off
const int potPin = A0;  // connected to the potentiometer's output

// create some variables to hold values from your inputs
int calcValue = 0;
int onOffSwitchState = 0;  // current state of the On/Off switch
int previousOnOffSwitchState = 0; // previous position of the on/off switch
int directionSwitchState = 0;  // current state of the direction switch
int previousDirectionSwitchState = 0;  // previous state of the direction switch

int motorEnabled = 0; // Turns the motor on/off
int motorSpeed = 0; // speed of the motor
int motorDirection = 1; // current direction of the motor

#include <PS4USB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
PS4USB PS4(&Usb);

int deadZoneUpper = (127+(255*deadZone)/2);
int deadZoneLower = (127-(255*deadZone)/2);

void setup() {
  // intialize the inputs and outputs
  pinMode(controlPin1, OUTPUT);
  pinMode(controlPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  // pull the enable pin LOW to start
  digitalWrite(enablePin, LOW);
  
  Serial.begin(115200);
  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect
  #endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 USB Library Started"));
}

void loop() {
  delay(1)
  Usb.Task();
  if (PS4.connected()) {

    // read the value of the on/off button
    onOffSwitchState = PS4.getButtonClick(CIRCLE);

    // if the on/off button changed state since the last loop()
    if (onOffSwitchState != previousOnOffSwitchState) {
      // change the value of motorEnabled if pressed
      if (onOffSwitchState) {
        motorEnabled = !motorEnabled;
      }
    }
    
    int rawValue = PS4.getAnalogHat(LeftHatY);
    //Serial.print("raw: "+String(rawValue)+"\n");
    if (rawValue > deadZoneUpper || rawValue < deadZoneLower) {
      calcValue = ((rawValue-127.5)/127.5)*255;
      //Serial.print("calc: "+String(calcValue)+"\n");
    } else {
      calcValue = 0;
    }
    
    // read the value of the direction button
    if (calcValue == 0) {
      motorDirection = 0;
    } else if (calcValue > 0) {
      motorDirection = -1;
    } else if (calcValue < 0) {
      motorDirection = 1;
    }
    
    // read the value of the pot and divide by 4 to get
    // a value that can be used for PWM
    motorSpeed = abs(calcValue);
    
    // change the direction the motor spins by talking
    // to the control pins on the H-Bridge
    if (motorDirection == 1) {
      Serial.print("forward\n");
      digitalWrite(controlPin1, HIGH);
      digitalWrite(controlPin2, LOW);
    } else if (motorDirection == -1) {
      Serial.print("backward\n");
      digitalWrite(controlPin1, LOW);
      digitalWrite(controlPin2, HIGH);
    } else {
      Serial.print("neutral\n");
      digitalWrite(controlPin1, LOW);
      digitalWrite(controlPin2, LOW);
    }
      analogWrite(enablePin, 0);
    }
  
    // if the motor is supposed to be on
    if (motorEnabled == 1) {
      // PWM the enable pin to vary the speed
      Serial.print(String(motorSpeed)+"\n");
      analogWrite(enablePin, motorSpeed);
    } else { // if the motor is not supposed to be on
      //turn the motor off
    // save the current On/Offswitch state as the previous
    previousDirectionSwitchState = directionSwitchState;
    // save the current switch state as the previous
    previousOnOffSwitchState = onOffSwitchState;
  }
}


