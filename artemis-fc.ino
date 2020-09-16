#include "MPU9250.h"
#include "SoftwareSerial.h"
#include <Servo.h>

MPU9250 mpu;
SoftwareSerial sbus = SoftwareSerial(A0, A5, true);
Servo motor;
Servo rudder;
Servo elevator;

float P_yaw = 1;
float I_yaw = 0;
float D_yaw = 0;
float P_pitch = -1;
float I_pitch = 0;
float D_pitch = 0;

int roll_sum = 0; //overflowに注意
int roll_before = 0;
int pitch_sum = 0;
int pitch_before = 0;

int rtemp1 = 0;
int rtemp2 = 0;

int etemp1 = 0;
int etemp2 = 0;

int cnt = 0;

float pitchZero = 0;
float rollZero = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED1,OUTPUT);
  pinMode(PIN_LED2,OUTPUT);
  digitalWrite(PIN_LED2,HIGH);

  sbus.begin(100000);
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  mpu.setup();
  motor.attach(A1);
  rudder.attach(A2);
  elevator.attach(A3);
  float pitchSum = 0;
  float rollSum = 0;
  const int repetition = 100;
  
  for(int i = 0; i< repetition; i++){
    mpu.update();
    pitchSum += mpu.getPitch();
    rollSum += mpu.getRoll();
  }

  pitchZero = pitchSum /repetition;
  rollZero = rollSum / repetition;

  digitalWrite(PIN_LED2,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  int dat[25];
  int d = sbus.read();
  int index = 0;
 
  while(d != -1){
    if(index < 25) dat[index] = d;
    index ++;
    d = sbus.read();
  }
  if(dat[0] == 240){
    if(dat[9] & 0x02 > 0){
      mpu.update();
      // センサーデータを取る
      digitalWrite(PIN_LED1,HIGH);
      int tau_roll = PIDcontrol(mpu.getRoll(),rollZero,0,&roll_before,P_yaw,I_yaw,D_yaw);
      int tau_pitch = PIDcontrol(mpu.getPitch(),pitchZero,0,&pitch_before,P_pitch,I_pitch,D_pitch);
      rudder.write(tau_roll+90);
      elevator.write(tau_pitch+90);
    }
    else{
      digitalWrite(PIN_LED1,LOW);
      if(index > 10){
          int rtemp = 180-((float)(((dat[6] & 0x0F) << 7)+((dat[5] & 0xFE) >> 1)-192)/1600*180);
          rudder.write((int)(rtemp + rtemp1 + rtemp2)/3);
          rtemp2 = rtemp1;
          rtemp1 = rtemp;
          
          int etemp = (float)(((dat[3] & 0x3F) << 5)+((dat[2] & 0xF8) >> 3)-192)/1600*180;
          elevator.write((int)(etemp + etemp1 + etemp2)/3);
        	etemp2 = etemp1;
  	      etemp1 = etemp;
  
          int mtemp = 180-((float)(((dat[5] & 0x01) << 10)+((dat[4] & 0xFF) << 2)+((dat[3] & 0xC0) >> 6)-192)/1600*180);
          motor.write(mtemp);
        }
      }
  }
}

// argument "current" should be the raw data from the sensor (therefore biased by the value set at calibration) and argument "target" should not be biased (therefore the argument should be 0 when completely horizontal). The return value is also biased.
int PIDcontrol(float current, float zero, int target, int *before, float P, float I, float D){
  current -= zero;
  float error = current - target;
  //sum += error;
  int dif = error-*before;
  *before = error;
  return (P*error+D*dif) + zero; //+I*sum
}
int P2control(int current_theta, int target_theta,int current_omega, int target_omega, int P_theta, int P_omega){
  int error_theta = current_theta - target_theta;
  int error_omega = current_omega - target_omega;
  return P_theta*error_theta+P_omega*error_omega;
}
