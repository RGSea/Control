/*
 Robert Gordon's College ROV Dualshock 4 Test Code

 Created 13 December 2016
 THIS TEST REQUIRES NOT GATES
 */

const float deadZone = 0.05; // range to which no input is read
const int controlPin1 = 2; // connected to Log1,3 & !Log2,4 on the H-bridge
const int enablePin = 9;   // connected to En1,2 on the H-bridge

// create some variables to hold values from your inputs
int calcValue = 0;

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
int inc = 0;

void setup() {
  // intialize the inputs and outputs
  pinMode(controlPin1, OUTPUT);
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
  delay(1);
  Usb.Task();
  if (PS4.connected()) {
    
   
    //Serial.print("raw: "+String(rawValue)+"\n");
    if (PS4.getButtonPress(R1)) {
      while (calcValue < 256) {
        inc = 1;
        calcValue += inc;
        inc += 0.1;
      }
      //Serial.print("calc: "+String(calcValue)+"\n");
    } else {
      calcValue = 0;
    }

    if (PS4.getButtonPress(L1)) {
      while (calcValue < 256){
        inc = -1;
        calcValue -= inc;
        inc -= -0.1;
      }
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
    } else if (motorDirection == -1) {
      Serial.print("backward\n");
      digitalWrite(controlPin1, LOW);
    } else {
      Serial.print("neutral\n");
      digitalWrite(controlPin1, LOW);
    }
    analogWrite(enablePin, 0);
    
    Serial.print(String(motorSpeed)+"\n");
    analogWrite(enablePin, motorSpeed);
    
  }
}


