byte testPins[] = {0};
byte n_pins = sizeof(testPins)/sizeof(byte);

void setup() {
  for (int i=0; i<n_pins; ++i) {
    pinMode(testPins[i], INPUT);
  }
}

void loop() {
  bool van = false;
  for (int i=0; i<n_pins; ++i) {
    byte buttonState = digitalRead(testPins[i]);
    if (buttonState == LOW) {
      Serial.print("{ ");
      Serial.print(testPins[i]);Serial.print(", LOW} ");
      van = true;
    } else {
      
    }
  }
  if (van) {
    Serial.print(" @ ");
    Serial.println(millis());
  }
}
