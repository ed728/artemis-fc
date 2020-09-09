#include "SoftwareSerial.h"
#include <Servo.h>
Servo rudder;
Servo elevator;
int rpast;

SoftwareSerial sbus = SoftwareSerial(A0, A5, true);

void setup() {
  // put your setup code here, to run once:
  sbus.begin(100000);
  Serial.begin(115200);
  rudder.attach(A2);
  elevator.attach(A3);
}
void loop() {
  int dat[25];
  int d = sbus.read();
  int index = 0;
  while(d != -1){
    if(index < 25) dat[index] = d;
    index ++;
    d = sbus.read();
  }
  if(index > 10){
    int rtemp = ((dat[6] & 0b00001111) << 4)+((dat[5] & 0b11110000) >> 4);
    if(rtemp - rpast > -3 && rtemp - rpast < 3){
      int v = (int)((float)rtemp/255*180);
      if(v != 134) rudder.write(v);
    }
    rpast = rtemp;
    
    /*int etemp = (float)(((dat[3] & 0x3F)<< 2)+((dat[2] & 0xC0) >> 6))/255*180;
    elevator.write(etemp);*/
  }
}
