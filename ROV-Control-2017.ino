//Robert gordons College ROV Contest
//Control Module code
//By Nimrod Libman,Neeley Corcoran,Murray Macfarlane 16/06/2016
#include <PS4USB.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
PS4USB PS4(&Usb);

const float deadZoneBuffer = 0.1; //if the absolute values of the x and y postion of the stick are withing this value of each other, stick is in deadzone.
//this is a part of the input library not this code


int vertIntensity; //PWM intensity for vertical motors. Set by input() every tick.
int lateIntensity; //PWM intensity for lateral  motors. Set by input() every tick.
char vertMotorDir; //determines whether we are going up, down, pitching up, pitching down, or rolling left or right. set by input() every tick.
char lateMotorDir; //determines whether we are going forward, back, left, right, or yawing left or right. set by input() every tick.


float zTranslationInput; // z if forward axis

float xTranslationInput; //x is right axis

float yRotationInput;
float yTranslationInput; //y is upwards axis

enum MotorIndices {
  MotorIndices_FrontMotor = 0,
  MotorIndices_BackMotor = 1,
  MotorIndices_LeftMotor = 2,
  MotorIndices_RightMotor = 3,
  MotorIndices_FrontLeftMotor = 4,
  MotorIndices_FrontRightMotor = 5,
  MotorIndices_BackLeftMotor = 6,
  MotorIndices_BackRightMotor = 7
};
enum MotorPins { //change CHNAGE MAN CHANGE AAAAA
  MotorPins_FrontMotor = 0,
  MotorPins_BackMotor = 1,
  MotorPins_LeftMotor = 2,
  MotorPins_RightMotor = 3,
  MotorPins_FrontLeftMotor = 4,
  MotorPins_FrontRightMotor = 5,
  MotorPins_BackLeftMotor = 6,
  MotorPins_BackRightMotor = 7
};

enum MotorPWM { //change CHNAGE MAN CHANGE AAAAA
  MotorPWM_FrontMotor = 8,
  MotorPWM_BackMotor = 9,
  MotorPWM_LeftMotor = 10,
  MotorPWM_RightMotor = 11,
  MotorPWM_FrontLeftMotor = 12,
  MotorPWM_FrontRightMotor = 13,
  MotorPWM_BackLeftMotor = 14,
  MotorPWM_BackRightMotor = 15
};


float motorDirections[8];

int grabberPin;
int grabberRotatePin;

bool deltaGrabber;

//  DIAGRAM OF JOYSTICK
//       +1
//      _______
//     / \   /  \
//    /   \ /    \
// -1|     .     |+1
//   |    / \    |
//    \  /   \  /
//     \_______/
//        -1




void setup() {
  deltaGrabber = false;
  // put your setup code here, to run once:
  Serial.begin(115200);
  //start up PS4 input

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 USB Library Started"));


  pinMode(MotorPins_FrontLeftMotor, OUTPUT);
  pinMode(MotorPins_FrontRightMotor, OUTPUT);
  pinMode(MotorPins_BackLeftMotor, OUTPUT);
  pinMode(MotorPins_BackRightMotor, OUTPUT);
  pinMode(MotorPins_LeftMotor, OUTPUT);
  pinMode(MotorPins_FrontRightMotor, OUTPUT);
  pinMode(MotorPins_BackMotor, OUTPUT);
  pinMode(MotorPins_BackMotor, OUTPUT);

  pinMode(MotorPWM_FrontLeftMotor, OUTPUT);
  pinMode(MotorPWM_FrontRightMotor, OUTPUT);
  pinMode(MotorPWM_BackLeftMotor, OUTPUT);
  pinMode(MotorPWM_BackRightMotor, OUTPUT);
  pinMode(MotorPWM_LeftMotor, OUTPUT);
  pinMode(MotorPWM_FrontRightMotor, OUTPUT);
  pinMode(MotorPWM_BackMotor, OUTPUT);
  pinMode(MotorPWM_BackMotor, OUTPUT);


  pinMode(grabberPin, OUTPUT);
  pinMode(grabberRotatePin, OUTPUT); //still need to add code to make it rotate


}




void loop() {

  delay(1);
  Usb.Task();
  if (PS4.connected()) {


    //get input
    zTranslationInput = (float)(PS4.getAnalogHat(LeftHatY)) / 128.0 - 0.5;
    xTranslationInput = (float)(PS4.getAnalogHat(LeftHatX)) / 128.0 - 0.5;


    yRotationInput = (float)(PS4.getAnalogHat(RightHatX)) / 128.0 - 0.5;
    yTranslationInput = (float)(PS4.getAnalogHat(RightHatY)) / 128.0 - 0.5;



    //deadzone snapping
    if (zTranslationInput < deadZoneBuffer && zTranslationInput > -deadZoneBuffer) {
      zTranslationInput = 0.0;
    }
    if (xTranslationInput < deadZoneBuffer && xTranslationInput > -deadZoneBuffer) {
      xTranslationInput = 0.0;
    }
    if (yTranslationInput < deadZoneBuffer && yTranslationInput > -deadZoneBuffer) {
      yTranslationInput = 0.0;
    }
    if (yRotationInput < deadZoneBuffer && yRotationInput > -deadZoneBuffer) {
      yRotationInput = 0.0;
    }



    //grab
    if (PS4.getButtonClick(DOWN)) {
      if (deltaGrabber == false) {
        digitalWrite(grabberPin, HIGH);
      }
      deltaGrabber = true;
    } else {
      if (deltaGrabber == true) {
        digitalWrite(grabberPin, LOW);
      }
      deltaGrabber = false;
    }


    motorDirections[0], motorDirections[1], motorDirections[2], motorDirections[3], motorDirections[4], motorDirections[5], motorDirections[6], motorDirections[7] = 0.0;






    //get required lateral inputs


    motorDirections[MotorIndices_FrontRightMotor] = -zTranslationInput;
    motorDirections[MotorIndices_FrontRightMotor] += xTranslationInput;
    motorDirections[MotorIndices_FrontRightMotor] += yRotationInput;

    if (abs(motorDirections[MotorIndices_FrontRightMotor]) / 2.0 > 1.0) {
      motorDirections[MotorIndices_FrontRightMotor] /= 3;
    } else if (abs(motorDirections[MotorIndices_FrontRightMotor]) > 1.0) {
      motorDirections[MotorIndices_FrontRightMotor] /= 2;
    }

    motorDirections[MotorIndices_FrontLeftMotor] = -zTranslationInput;
    motorDirections[MotorIndices_FrontLeftMotor] -= xTranslationInput;
    motorDirections[MotorIndices_FrontLeftMotor] -= yRotationInput;

    if (abs(motorDirections[MotorIndices_FrontLeftMotor]) / 2.0 > 1.0) {
      motorDirections[MotorIndices_FrontLeftMotor] /= 3;
    } else if (abs(motorDirections[MotorIndices_FrontLeftMotor]) > 1.0) {
      motorDirections[MotorIndices_FrontLeftMotor] /= 2;
    }


    motorDirections[MotorIndices_BackRightMotor] = zTranslationInput;
    motorDirections[MotorIndices_BackRightMotor] += xTranslationInput;
    motorDirections[MotorIndices_BackRightMotor] -= yRotationInput;

    if (abs(motorDirections[MotorIndices_BackRightMotor]) / 2.0 > 1.0) {
      motorDirections[MotorIndices_BackRightMotor] /= 3;
    } else if (abs(motorDirections[MotorIndices_BackRightMotor]) > 1.0) {
      motorDirections[MotorIndices_BackRightMotor] /= 2;
    }



    motorDirections[MotorIndices_BackLeftMotor] = zTranslationInput;
    motorDirections[MotorIndices_BackLeftMotor] -= xTranslationInput;
    motorDirections[MotorIndices_BackLeftMotor] += yRotationInput;

    if (abs(motorDirections[MotorIndices_BackLeftMotor]) / 2.0 > 1.0) {
      motorDirections[MotorIndices_BackLeftMotor] /= 3;
    } else if (abs(motorDirections[MotorIndices_BackLeftMotor]) > 1.0) {
      motorDirections[MotorIndices_BackLeftMotor] /= 2;
    }




    //now for vertical motors
    motorDirections[MotorIndices_FrontMotor] = yTranslationInput;


    motorDirections[MotorIndices_BackMotor] = -yTranslationInput;


    motorDirections[MotorIndices_LeftMotor] = -yTranslationInput;


    motorDirections[MotorIndices_RightMotor] = -yTranslationInput;



  }




  //now set motor logic pins
  if (motorDirections[MotorIndices_FrontLeftMotor] > 0.0) {
    digitalWrite(MotorPins_FrontLeftMotor, HIGH);
  }
  else {
    digitalWrite(MotorPins_FrontLeftMotor, LOW);
  }

  if (motorDirections[MotorIndices_FrontRightMotor] > 0.0) {
    digitalWrite(MotorPins_FrontRightMotor, HIGH);
  }
  else {
    digitalWrite(MotorPins_FrontRightMotor, LOW);
  }

  if (motorDirections[MotorIndices_FrontLeftMotor] > 0.0) {
    digitalWrite(MotorPins_FrontLeftMotor, HIGH);
  }
  else {
    digitalWrite(MotorPins_FrontLeftMotor, LOW);
  }

  if (motorDirections[MotorIndices_BackRightMotor] > 0.0) {
    digitalWrite(MotorPins_BackRightMotor, HIGH);
  }
  else {
    digitalWrite(MotorPins_BackRightMotor, LOW);
  }

  //vertical motors
  if (motorDirections[MotorIndices_LeftMotor] > 0.0) {
    digitalWrite(MotorPins_LeftMotor, HIGH);
  }
  else {
    digitalWrite(MotorPins_LeftMotor, LOW);
  }

  if (motorDirections[MotorIndices_RightMotor] > 0.0) {
    digitalWrite(MotorPins_RightMotor, HIGH);
  }
  else {
    digitalWrite(MotorPins_RightMotor, LOW);
  }

  if (motorDirections[MotorIndices_FrontMotor] > 0.0) {
    digitalWrite(MotorPins_FrontMotor, HIGH);
  }
  else {
    digitalWrite(MotorPins_FrontMotor, LOW);
  }

  if (motorDirections[MotorIndices_RightMotor] > 0.0) {
    digitalWrite(MotorPins_BackMotor, HIGH);
  }
  else {
    digitalWrite(MotorPins_BackMotor, LOW);
  }


  //write PWM
  analogWrite(MotorPWM_FrontLeftMotor, abs(motorDirections[MotorIndices_FrontLeftMotor]) * 255);
  analogWrite(MotorPWM_FrontRightMotor, abs(motorDirections[MotorIndices_FrontRightMotor]) * 255);
  analogWrite(MotorPWM_BackLeftMotor, abs(motorDirections[MotorIndices_BackLeftMotor]) * 255);
  analogWrite(MotorPWM_BackRightMotor, abs(motorDirections[MotorIndices_BackRightMotor]) * 255);

  analogWrite(MotorPWM_FrontMotor, abs(motorDirections[MotorIndices_FrontMotor]) * 255);
  analogWrite(MotorPWM_RightMotor, abs(motorDirections[MotorIndices_RightMotor]) * 255);
  analogWrite(MotorPWM_LeftMotor, abs(motorDirections[MotorIndices_LeftMotor]) * 255);
  analogWrite(MotorPWM_BackMotor, abs(motorDirections[MotorIndices_BackMotor]) * 255);


}

