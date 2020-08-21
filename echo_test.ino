
int trigger_pin = 11;
int echo_pin = 7;

long dur;
int dist;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);
  pinMode(trigger_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);

  dur = pulseIn(echo_pin, HIGH);
  dist = (dur*0.034)/2.0;

  Serial.print("Dist: ");
  Serial.print(dist);
  Serial.print(" cm");
  Serial.println();

  delay(10);
}
