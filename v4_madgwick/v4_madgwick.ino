#include "Wire.h"
#include <AX12A.h> 
#include <TroykaIMU.h>
#include "math.h"
#include <I2Cdev.h>

#define DirectionPin (10u) 
#define BaudRate (1000000ul) 
#define ID_1 (1u) 
#define ID_2 (2u) 
#define ID_3 (3u)

#define Kp  25  /*Too small Kp will cause the robot to fall, because the fix is ​​not enough. 
Too much Kp forces the robot to go wildly forward and backward.
A good Kp will make the robot move very little back and forth (or slightly oscillates).*/
#define Kd  0.02 /*A good Kd value will reduce the vibrations until the robot becomes almost steady. 
In addition, the correct Kd will hold the robot, even if it is pushed. */
#define Ki  5  /*The correct Ki value will shorten the time required to stabilize the robot.*/
#define sampleTime  0.0005 /*время выборки*/
#define targetAngle -0.5 /*целевой угол*/

#define BETA 0.22f

Madgwick filter;

Accelerometer accel;
Gyroscope gyro;

float gx, gy, gz, ax, ay, az, yaw, pitch, roll;
float fps = 100;
int motorPower, gyroRate;
float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;

void setMotors(int ID_1speed, int ID_3speed) {
  if(ID_1speed >= 0) {
    ax12a.turn(ID_1, RIGHT, 100);
  }
  else {
    //ax12a.turn(ID_1, LEFT, motorPower);
    ax12a.turn(ID_1, LEFT, 100);
  }
  if(ID_3speed >= 0) {
    ax12a.turn(ID_3, LEFT, 100);
  }
  else {
    //ax12a.turn(ID_3, RIGHT, motorPower);
    ax12a.turn(ID_3, RIGHT, 100);
  }
}

void init_PID() 
{  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}


void setup() 
{ 
  Serial.begin(1000000);
  ax12a.begin(BaudRate, DirectionPin, &Serial); 
  ax12a.setEndless(ID_1, ON); 
  ax12a.setEndless(ID_3, ON); 
  ax12a.torqueStatus(ID_2,ON);
  accel.begin();
  gyro.begin();
  init_PID();
} 

void loop() 
{
  ax12a.torqueStatus(ID_2, ON);
  accel.readGXYZ(&ax, &ay, &az);
  gyro.readRadPerSecXYZ(&gx, &gy, &gz);
  filter.setKoeff(fps, BETA);
  filter.update(gx, gy, gz, ax, ay, az);
  setMotors(motorPower, motorPower);
  //yaw =  filter.getYawDeg();
  //pitch = filter.getPitchDeg();
  roll = filter.getRollDeg();
  delay(1);
}
ISR(TIMER1_COMPA_vect)
{ 
  currentAngle = roll;
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //motorPower = map((Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime),-1023, 1023, -50, 50);
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
}
