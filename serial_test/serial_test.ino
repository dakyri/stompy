byte commandByte;
byte noteByte;
byte velocityByte;

byte clockMidi = 0xf8;

//light up led at pin 13 when receiving noteON message with note = 60

void setup(){
  Serial1.begin(31250);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
}

void checkMIDI(){
  do{
    if (Serial1.available()){
      commandByte = Serial1.read();//read first byte
      noteByte = Serial1.read();//read next byte
      velocityByte = Serial1.read();//read final byte
      Serial.println(commandByte, HEX);
      if (commandByte == clockMidi){//if clockMidi on message
        digitalWrite(13,HIGH);//turn on led
      }
    }
  }
  while (Serial1.available() > 2);//when at least three bytes available
}
    

void loop(){
  checkMIDI();
  delay(100);
  digitalWrite(13,LOW);//turn led off
}

