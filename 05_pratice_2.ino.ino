int red = 7;
long time = millis();

void setup() {
  pinMode(red, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  while (time + 1000 > millis()) {
    digitalWrite(7, HIGH);
  }
  while (time + 2000 > millis()) {
    digitalWrite(7, LOW);
    delay(125);
    digitalWrite(7, HIGH);
    delay(125);
  }
  while(1) {
    digitalWrite(7, LOW);
}
}
