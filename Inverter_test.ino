#include <Arduino.h>

int inPin = 7;
int outPin = 27;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(inPin, INPUT_PULLDOWN);  // sets the digital pin 13 as output
  pinMode(outPin, OUTPUT);

  Serial.println("Starting.");
  //digitalWrite(outPin, HIGH);
}

void loop() {
  digitalWrite(outPin, LOW);
  // put your main code here, to run repeatedly:
  Serial.print(digitalRead(inPin));
  Serial.print(" ");
  Serial.println();  
}
