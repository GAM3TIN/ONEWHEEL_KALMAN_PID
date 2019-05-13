/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Servo.h>

Servo HubMotor;

#define RESTRICT_PITCH 

Kalman kalmanX; // Create the Kalman instances
//Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double dt=0;

float angle_estimate=0;
double throttle=1500; //initial value of throttle to the motors
float desired_angle = 0;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

//////////////////////Foot Button////////////////////////
float presstotal=0;
int left = A0;
int right = A1;
float pressleft;
float pressright; 
//int FSR=0;
//int turnon=0;
/////////////////////////////////////////////////////////
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;  // P constant Reconmend 3.55
float pid_i=0;  // I constant Reconmend 0.003
float pid_d=0;  // D constant Reconmend 2.05
/////////////////PID CONSTANTS/////////////////
double kp=3.55;//Adjust me to adjust PID Proportional
double ki=0.005;//Adjust me to adjust PID Integral
double kd=2.05;//Adjust me to adjust PID Derivitive
///////////////////////////////////////////////

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


int count=0;

// TODO: Make calibration routine

void setup() {
  SerialUSB.begin(0);

//delay(100);
  //SerialUSB.println("hi!!!");



  Wire.begin();
 #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif


  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    SerialUSB.print(F("Error reading sensor"));
    while (1);
  }

  HubMotor.attach(3); //attatch the Hub motor to pin 3
  HubMotor.writeMicroseconds(1500);

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  //double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  //double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  //kalmanY.setAngle(pitch);
  gyroXangle = roll;
  //gyroYangle = pitch;
  compAngleX = roll;
  //compAngleY = pitch;

  timer = micros();
}

void loop() {

  //SerialUSB.println("hi!!!");
  
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  //double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  //double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  //double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

/*
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
*/

 // gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
//  gyroYangle += gyroYrate * dt;
 gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  //compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  //if (gyroYangle < -180 || gyroYangle > 180)
  //  gyroYangle = kalAngleY;

  /* Print Data */
#if 1 // Set to 1 to activate
  //SerialUSB.print(accX); SerialUSB.print("\t");
  //SerialUSB.print(accY); SerialUSB.print("\t");
  //SerialUSB.print(accZ); SerialUSB.print("\t");

  //SerialUSB.print(gyroX); SerialUSB.print("\t");
 // SerialUSB.print(gyroY); SerialUSB.print("\t");
 // SerialUSB.print(gyroZ); SerialUSB.print("\t");

  //SerialUSB.print("\t");
#endif

  //Serial.print(dt); Serial.print("\t");

  //SerialUSB.print(roll); SerialUSB.print("\t");
  //SerialUSB.print(gyroXangle); SerialUSB.print("\t");
  //SerialUSB.print(compAngleX); SerialUSB.print("\t");
  //SerialUSB.print(kalAngleX); SerialUSB.print("\t"); 

  //SerialUSB.print("\t");

 // SerialUSB.print(pitch); SerialUSB.print("\t");
 // SerialUSB.print(gyroYangle); SerialUSB.print("\t");
 // SerialUSB.print(compAngleY); SerialUSB.print("\t");
  //SerialUSB.print(kalAngleY); SerialUSB.print("\t");

#if 0 // Set to 1 to print the temperature
  //SerialUSB.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  //SerialUSB.print(temperature); SerialUSB.print("\t");
#endif
/////////////////////Foot button///////////////////////
pressright=(float)analogRead(right);                //reading pin to see the voltage
pressleft=(float)analogRead(left);                 //reading pin to see the voltage
//pressright=pressright-1.65;
//pressright=pressright/1.35;             //Hook sencors to 3.3volts then the other lead to A0 for left side and A1 for the right side
//pressleft=pressleft-1.65;    
//pressleft=pressleft/1.35;              //if the button is not pressed then presstotal = 3 volts if button fully pressed then it is 1.65v  range for voltage now is 0-1volts
//pressleft=1-pressleft;                  //revercing the range y=(range)-x EXAMPLE: range 1,0 input=0.2 OUTPUT=0.8               
//pressright=1-pressright;
//Serial.println(pressright);
//Serial.println(pressleft);
//presstotal=pressright-pressleft;        // Now our range is between -1,1 that being our output                                                        
//presstotal=presstotal*400;      // Making the max inflence of this between -400,400us going to the VESC (Side note, tune VESC to have max power at 2000 and max back at 1000) 


                                          
///////////////////////////////////////////////////////

//PID Control


angle_estimate = kalAngleX;

/*///////////////////////////P I D///////////////////////////////////*/
/*Remember that for the balance we will use just one axis. I've choose the x angle
to implement the PID with. That means that the x axis of the IMU has to be paralel to
the balance*/

/*First calculate the error between the desired angle and 
*the real measured angle*/
error = angle_estimate - desired_angle;
    
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pid_p = kp*error;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if ((error>-3)&&(error<3))
{
  pid_i = pid_i+(ki*error);  
}

/*The last part is the derivative. The derivative acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For that we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time. 
Finnaly we multiply the result by the derivate constant*/

pid_d = kd*((error - previous_error)/dt);

/*The final PID values is the sum of each of this 3 parts*/
PID = pid_p + pid_i + pid_d + presstotal;  //Adding PID and a aditional command for moving 
                                           //forward or backward

/*We know that the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PID < -1000)    // PID Must be within -1000 and 1000
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
//pwmLeft = throttle + PID;
pwmRight = throttle - PID;  //1500 (dead stop) +- 500

/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
if(pwmRight < 1000)   //we cant have numbers like 500 of 2500 so this brings it to normal
{
  pwmRight= 1000;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}




if(((pressright+pressleft)<1) && (count==0)){   //change the need for foot pads when testing/tuneing) 
  delay(1000);
  count=count+1;
}

if(count>0){
HubMotor.writeMicroseconds(pwmRight);
}
previous_error = error; //Remember to store the previous error.

if(count==10){ //making sure that our one time use counter dose not get past 10 (to pervent craching at 1*10^14)
count=1;
}
  
  delay(10);
}
