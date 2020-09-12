#include "SoftwareSerial.h"
#include <Servo.h>
#include "MPU9250.h"

MPU9250 mpu;
SoftwareSerial sbus = SoftwareSerial(A0, A5, true);
Servo motor;
Servo rudder;
Servo elevator;

int P_yaw = 2;
int I_yaw = 0;
int D_yaw = 0;
int P_pitch = -4;
int I_pitch = 0;
int D_pitch = 0;

int roll_sum = 0; //overflowに注意
int roll_before = 0;
int pitch_sum = 0;
int pitch_before = 0;

void setup() {
  // put your setup code here, to run once:
  sbus.begin(100000);
  Wire.begin();
  delay(2000);
  mpu.setup();
  motor.attach(A1);
  rudder.attach(A2);
  elevator.attach(A3);
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
  if(dat[9] & 0x02 > 0){
    // センサーデータを取る
    mpu.update();
    int tau_roll = PIDcontrol(mpu.getRoll(),0,&roll_before,P_yaw,I_yaw,D_yaw);
    int tau_pitch = PIDcontrol(mpu.getPitch(),0,&pitch_before,P_pitch,I_pitch,D_pitch);
    rudder.write(tau_roll+90);
    elevator.write(tau_pitch+90);
  }
  else{
    if(index > 10){
      if(dat[0] == 240){
        int rtemp = ((dat[6] & 0x0F) << 4)+((dat[5] & 0xF0) >> 4);
        rudder.write(rtemp);
        
        int etemp = ((dat[3] & 0x3F)<< 2)+((dat[2] & 0xC0) >> 6);
        elevator.write(etemp);

        int mtemp = ((dat[5] & 0x01) << 7)+((dat[4] & 0xFE) >> 1);
        motor.write(255-mtemp);
      }
    }
  }
}

int PIDcontrol(int current, int target, int *before, int P, int I, int D){
  int error = current - target;
  //sum += error;
  int dif = error-*before;
  *before = error;
  return P*error+D*dif; //+I*sum
}
int P2control(int current_theta, int target_theta,int current_omega, int target_omega, int P_theta, int P_omega){
  int error_theta = current_theta - target_theta;
  int error_omega = current_omega - target_omega;
  return P_theta*error_theta+P_omega*error_omega;
}
