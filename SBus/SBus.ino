#include "SoftwareSerial.h"

SoftwareSerial sbus = SoftwareSerial(7, 15, true);
int count;
long interval;

void setup() {
  // put your setup code here, to run once:
  sbus.begin(100000);
  Serial.begin(115200);
  count=0;
}

void loop() {
  int data[26];
  int val[19];
  int i;
  if (sbus.available() > 0) {
    data[count]=sbus.read();
    interval=millis();
    count++;
  }
  if ((interval+4 < millis()) && (0 < count) ) {
    count=0;
  
   val[0] =((data[1] & 0xff)<<0) + ((data[2] & 0x07)<<8);
   val[1] =((data[2] & 0xf8)>>3) + ((data[3] & 0x3f)<<5);
   val[2] =((data[3] & 0xc0)>>6) + ((data[4] & 0xff)<<2) + ((data[5] & 0x01)<<10);
   val[3] =((data[5] & 0xfe)>>1) + ((data[6] & 0x0f)<<7);
   val[4] =((data[6] & 0x0f)>>4) + ((data[7] & 0x7f)<<4);
   val[5] =((data[7] & 0x80)>>7) + ((data[8] & 0xff)<<1) + ((data[9] & 0x03) <<9);
   val[6] =((data[9] & 0x7c)>>2) + ((data[10] & 0x1f)<<6);
   val[7] =((data[10] & 0xe0)>>5) + ((data[11] & 0xff)<<3);
  
   val[8] =((data[12] & 0xff)<<0) + ((data[13] & 0x07)<<8);
   val[9] =((data[13] & 0xf8)>>3) + ((data[14] & 0x3f)<<5);
   val[10]=((data[14] & 0xc0)>>6) + ((data[15] & 0xff)<<2) + ((data[16] & 0x01)<<10);
   val[11]=((data[16] & 0xfe)>>1) + ((data[17] & 0x0f)<<7);
   val[12]=((data[17] & 0x0f)>>4) + ((data[18] & 0x7f)<<4);
   val[13]=((data[18] & 0x80)>>7) + ((data[19] & 0xff)<<1) + ((data[20] & 0x03) <<9);
   val[14]=((data[20] & 0x7c)>>2) + ((data[21] & 0x1f)<<6);
   val[15]=((data[21] & 0xe0)>>5) + ((data[22] & 0xff)<<3);
   val[16] = (data[23] & 0x1) ? 0x7ff : 0 ;
   val[17] = (data[23] & 0x2) ? 0x7ff : 0 ;
   val[18] = (data[23] & 0x8) ? 0x7ff : 0 ; // Failsafe

   for (i=0 ; i<19; i++ ) {
     Serial.print(val[i],DEC);
     Serial.print(F(" "));
   }
    Serial.print(F("\n"));
   }
}
