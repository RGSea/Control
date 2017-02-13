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


float zTranslationInput;
float zRotationInput;

float xRotationInput;
float xTranslationInput;

float yRotationInput;
float yTranslationInput;

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


//gyro variables

#include <Wire.h>
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int Serial_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;




void setup() {
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



  //start up MPU6050
  Wire.begin();

  Serial.print("registering gyro");
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  Serial.print("Calibrating gyro");                                       //Print text to screen

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(2);
    //Delay 3us to simulate the 250Hz program loop, might need to do less due to the REST of the code
    Serial.print(cal_int);
    Serial.print(F("\r\n"));
  }
  //get average offset
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;

  Serial.print("Done calibrating");


  loop_timer = micros();                                               //Reset the loop timer

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

}




void loop() {

  delay(1);
  Usb.Task();
  if (PS4.connected()) {
    //check for gyro correction


    if (PS4.getButtonClick(CROSS)) {
      Serial.print(F("\r\nCross"));
      PS4.setLedFlash(10, 10); // Set it to blink rapidly
      if (angle_pitch_output / 10 > 1) {
        xRotationInput = 1;
      } else if (angle_pitch_output / 10 < -1) {
        xRotationInput = -1;
      } else {
        xRotationInput = angle_pitch_output / 10;
      }

      if (angle_roll_output / 10 > 1) {
        zRotationInput = 1;
      } else if (angle_roll_output / 10 < -1) {
        zRotationInput = -1;
      } else {
        zRotationInput = angle_roll_output / 10;
      }

      //lock input

      yRotationInput = 0;

      xTranslationInput = 0;
      yTranslationInput = 0;
      zTranslationInput = 0;

    } else {

      //get input
      zTranslationInput = (float)(PS4.getAnalogHat(LeftHatY)) / 128.0 - 0.5;
      xTranslationInput = (float)(PS4.getAnalogHat(LeftHatX)) / 128.0 - 0.5;

      xRotationInput = (float)(PS4.getAnalogHat(RightHatY)) / 128.0 - 0.5;

      if (PS4.getButtonClick(R3)) {
        yRotationInput = 0.0;
        zRotationInput = (float)(PS4.getAnalogHat(RightHatX)) / 128.0 - 0.5; //but could be the  x translation
      } else {
        yRotationInput = (float)(PS4.getAnalogHat(RightHatX)) / 128.0 - 0.5; //but could be the  x translation
        zRotationInput = 0.0;
      }

      // up down
      yTranslationInput = ((float)PS4.getAnalogButton(R2) / 255.0 + (float)PS4.getAnalogButton(L2) / 255.0) / 2.0 - 1.0;



      //deadzone snapping
      if (zTranslationInput < deadZoneBuffer && zTranslationInput > -deadZoneBuffer) {
        zTranslationInput = 0.0;
      }
      if (zRotationInput < deadZoneBuffer && zRotationInput > -deadZoneBuffer) {
        zRotationInput = 0.0;
      }
      if (xTranslationInput < deadZoneBuffer && xTranslationInput > -deadZoneBuffer) {
        xTranslationInput = 0.0;
      }
      if (xRotationInput < deadZoneBuffer && xRotationInput > -deadZoneBuffer) {
        xRotationInput = 0.0;
      }
      if (yTranslationInput < deadZoneBuffer && yTranslationInput > -deadZoneBuffer) {
        yTranslationInput = 0.0;
      }
      if (yRotationInput < deadZoneBuffer && yRotationInput > -deadZoneBuffer) {
        yRotationInput = 0.0;
      }
    }
    ///gyro and grabber stuff

    if (PS4.getButtonClick(UP)) {
      Serial.print(F("\r\nUp"));
      PS4.setLed(Red);
    } if (PS4.getButtonClick(RIGHT)) {
      Serial.print(F("\r\nRight"));
      PS4.setLed(Blue);
    } if (PS4.getButtonClick(DOWN)) {
      Serial.print(F("\r\nDown"));
      PS4.setLed(Yellow);
    } if (PS4.getButtonClick(LEFT)) {
      Serial.print(F("\r\nLeft"));
      PS4.setLed(Green);
    }

    //get gyro data
    read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

    gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
    gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
    gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
    angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
    angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
    angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

    if (set_gyro_angles) {                                               //If the IMU is already started
      angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
      angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    }
    else {                                                               //At first start
      angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
      angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
      set_gyro_angles = true;                                            //Set the IMU started flag
    }

    //To dampen the pitch and roll angles a complementary filter is used
    angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
    angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

    write_Serial();                                                         //Write the roll and pitch values to the Serial display

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
    //we have y-translation, x-rotation, and z-rotation

    motorDirections[MotorIndices_FrontMotor] = yTranslationInput;
    motorDirections[MotorIndices_FrontMotor] -= xRotationInput;
    //motorDirections[MotorIndices_FrontMotor] += zRotationInput;

    if (abs(motorDirections[MotorIndices_FrontMotor]) > 1.0) {
      motorDirections[MotorIndices_FrontMotor] /= 2;
    }


    motorDirections[MotorIndices_BackMotor] = -yTranslationInput;
    motorDirections[MotorIndices_BackMotor] += xRotationInput;
    //motorDirections[MotorIndices_BackMotor] += zRotationInput;

    if (abs(motorDirections[MotorIndices_BackMotor]) > 1.0) {
      motorDirections[MotorIndices_BackMotor] /= 2;
    }

    motorDirections[MotorIndices_LeftMotor] = -yTranslationInput;
    //motorDirections[MotorIndices_LeftMotor] += xRotationInput;
    motorDirections[MotorIndices_LeftMotor] += zRotationInput;

    if (abs(motorDirections[MotorIndices_LeftMotor]) > 1.0) {
      motorDirections[MotorIndices_LeftMotor] /= 2;
    }

    motorDirections[MotorIndices_RightMotor] = -yTranslationInput;
    //motorDirections[MotorIndices_RightMotor] += xRotationInput;
    motorDirections[MotorIndices_RightMotor] -= zRotationInput;

    if (abs(motorDirections[MotorIndices_RightMotor]) > 1.0) {
      motorDirections[MotorIndices_RightMotor] /= 2;
    }
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


  while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer

}



void read_mpu_6050_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable
  Serial.print(F("read data"));
}

void write_Serial() {
  angle_pitch_buffer = angle_pitch_output * 10;
  Serial.print(F("Pitch:"));
  Serial.print(angle_pitch_output);


  angle_roll_buffer = angle_roll_output * 10;
  Serial.print(F("Roll:"));
  Serial.print(angle_roll_output);


  Serial.print(F("\r\n"));
}

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}


