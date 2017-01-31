//Robert gordons College ROV Contest
//Control Module code
//By Nimrod Libman,Neeley Corcoran 16/06/2016





//const float deadZoneBuffer = 0.1; //if the absolute values of the x and y postion of the stick are withing this value of each other, stick is in deadzone.
//this is a part of the input library not this code


int vertIntensity; //PWM intensity for vertical motors. Set by input() every tick.
int lateIntensity; //PWM intensity for lateral  motors. Set by input() every tick.
char vertMotorDir; //determines whether we are going up, down, pitching up, pitching down, or rolling left or right. set by input() every tick.
char lateMotorDir; //determines whether we are going forward, back, left, right, or yawing left or right. set by input() every tick.


#include <input.h> //include input library. This will make use of the above globals to manipulate the control.

Input input;

enum Pins { //PINOUTs for the motors. Replace witha actual PINOUT diagram
  LFL,
  LFR,
  LBL,
  LBR,
  VFL,
  VFR,
  VBL,
  VBR,
  VPWM,
  LPWM,
}



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
  // put your setup code here, to run once:
  Serial.begin(9600);
  input = new Input();//defined by input library
}

void loop() {
  // put your main code here, to run repeatedly:
  input.LoopCode();//defined by the input library.
//then assign vertIntesity and all that via the fucntions.
  //vertcial motor control
  analogWrite(VPWM, vertIntensity); //set PWM
  switch (vertMotorDir) {
    case ("U"): //up
      digitalWrite(VFL, 0);
      digitalWrite(VFR, 0);
      digitalWrite(VBL, 0);
      digitalWrite(VBR, 0);
      break;
    case ("D"): //down
      digitalWrite(VFL, 1);
      digitalWrite(VFR, 1);
      digitalWrite(VBL, 1);
      digitalWrite(VBR, 1);
      break;
    case ("L"): //left roll
      digitalWrite(VFL, 1);
      digitalWrite(VFR, 0);
      digitalWrite(VBL, 1);
      digitalWrite(VBR, 0);
      break;
    case ("R"): //right roll
      digitalWrite(VFL, 0);
      digitalWrite(VFR, 1);
      digitalWrite(VBL, 0);
      digitalWrite(VBR, 1);
      break;
    case ("S"): //pitch up (SURFACE)
      digitalWrite(VFL, 0);
      digitalWrite(VFR, 0);
      digitalWrite(VBL, 1);
      digitalWrite(VBR, 1);
      break;
    case ("G"): //pitch down (GROUND)
      digitalWrite(VFL, 1);
      digitalWrite(VFR, 1);
      digitalWrite(VBL, 0);
      digitalWrite(VBR, 0);
      break;
    default://should not go here. SERIOULY ACTUALLY MAKE THE INPUT LIBRARY GOOD.

      break;
  }

  //lateral motor control
  analogWrite(LPWM, lateIntensity); //set PWM
  switch (vertMotorDir) {
    case ("F"): //forward
      digitalWrite(LFL, 1);
      digitalWrite(LFR, 1);
      digitalWrite(LBL, 1);
      digitalWrite(LBR, 1);
      break;
    case ("B"): //backward
      digitalWrite(LFL, 0);
      digitalWrite(LFR, 0);
      digitalWrite(LBL, 0);
      digitalWrite(LBR, 0);
      break;
    case ("L"): //left strafe
      digitalWrite(LFL, 1);
      digitalWrite(LFR, 0);
      digitalWrite(LBL, 0);
      digitalWrite(LBR, 1);
      break;
    case ("R"): //right strafe
      digitalWrite(LFL, 0);
      digitalWrite(LFR, 1);
      digitalWrite(LBL, 1);
      digitalWrite(LBR, 0);
      break;
    case ("S"): //yaw right (STARBOARD)
      digitalWrite(LFL, 1);
      digitalWrite(LFR, 0);
      digitalWrite(LBL, 1);
      digitalWrite(LBR, 0);
      break;
    case ("P"): //yaw left (PORT)
      digitalWrite(LFL, 0);
      digitalWrite(LFR, 1);
      digitalWrite(LBL, 0);
      digitalWrite(LBR, 1);
      break;
    default://should not go here. SERIOULY ACTUALLY MAKE THE INPUT LIBRARY GOOD.

      break;
  }

 


}
