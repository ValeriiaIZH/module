#include "Wire.h"
#include <AX12A.h> 
#include <TroykaIMU.h>
#include "math.h"
#include <I2Cdev.h>

#define Direction_pin (10u) 
#define Baud_rate (1000000ul) 
#define ID_1 (1u) 
#define ID_2 (2u) 
#define ID_3 (3u)

#define Kp  25  /*Too small Kp will cause the robot to fall, because the fix is ​​not enough. 
Too much Kp forces the robot to go wildly forward and backward.
A good Kp will make the robot move very little back and forth (or slightly oscillates).*/
#define Kd  0.02 /*A good Kd value will reduce the vibrations until the robot becomes almost steady. 
In addition, the correct Kd will hold the robot, even if it is pushed. */
#define Ki  5  /*The correct Ki value will shorten the time required to stabilize the robot.*/
#define sample_time  0.0005 /*время выборки*/
#define target_angle -1.5 /*целевой угол*/

#define BETA 0.22f

Madgwick filter;
Accelerometer accel;
Gyroscope gyro;

float gx, gy, gz, ax, ay, az, roll;
float fps = 100;
int motor_power;
float  current_angle, prev_angle=0, error, prev_error=0, error_sum=0;
unsigned long delay_time = 0;

//speed of stabilization
void set_motors(int motor_1, int motor_2, int speed_1, int speed_3) {
  if(motor_1 >= 0) {
    ax12a.turn(ID_1, RIGHT, speed_1);
  }
  else {
    ax12a.turn(ID_1, LEFT, speed_1);
  }
  if(motor_2 >= 0) {
    ax12a.turn(ID_3, LEFT, speed_3);
  }
  else {
    ax12a.turn(ID_3, RIGHT, speed_3);
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

//moviement functions
void go_forward(){
ax12a.turn(ID_3, LEFT,100); 
ax12a.turn(ID_1, RIGHT, 100);
}

void go_backward(){
ax12a.turn(ID_3, RIGHT, 100); 
ax12a.turn(ID_1, LEFT, 100);
}

void turn_left(){
//ax12a.turn(ID_3, RIGHT, 0);  //another way to turn
ax12a.turn(ID_3, RIGHT, 100); 
ax12a.turn(ID_1, RIGHT, 100);
}

void turn_right(){
ax12a.turn(ID_3, LEFT, 100); 
ax12a.turn(ID_1, LEFT, 100);
//ax12a.turn(ID_1, LEFT, 0); //another way to turn
}

void stop_movement(){
ax12a.turn(ID_3, RIGHT, 0); 
ax12a.turn(ID_1, RIGHT, 0);
//set_motors(motor_power, motor_power, 100, 100);

}

//path function
void path(){
  if(millis() - delay_time <= 5000){
      go_forward();
    }
    if (millis() - delay_time > 5000){
      stop_movement(); 
    }
   // else delay_time = millis();  // if turn on this string the module will work like go->stop->go->stop->...etc
}

void setup() 
{ 
  Serial.begin(1000000);
  ax12a.begin(Baud_rate, Direction_pin, &Serial); 
  ax12a.setEndless(ID_1, ON); 
  ax12a.setEndless(ID_3, ON); 
  ax12a.torqueStatus(ID_2,ON);
  accel.begin();
  gyro.begin();
  init_PID();
} 

void loop() 
{
  //delay_time = millis();
  ax12a.torqueStatus(ID_2, ON);
  //get data from IMU
  accel.readGXYZ(&ax, &ay, &az);
  gyro.readRadPerSecXYZ(&gx, &gy, &gz);
  //call Madgwick filter
  filter.setKoeff(fps, BETA);
  filter.update(gx, gy, gz, ax, ay, az);
  roll = filter.getRollDeg();
  //look at roll's value 
  Serial.println(roll);
  if( -3.5 > roll || roll > 2 ){
        set_motors(motor_power, motor_power, 250, 250);
    } 
    else path();
}
 
ISR(TIMER1_COMPA_vect)
{ 
  current_angle = roll;  
  error = current_angle - target_angle;
  error_sum = error_sum + error;  
  error_sum = constrain(error_sum, -300, 300);
  motor_power = Kp*(error) + Ki*(error_sum)*sample_time - Kd*(current_angle-prev_angle)/sample_time;
  prev_angle = current_angle;
}
