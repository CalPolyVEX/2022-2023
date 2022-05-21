#define encoder1A 2
#define encoder1B 12
#define encoder2A 3
#define encoder2B 13
#define encoder3A 18
#define encoder3B 28
#define ppr 100
#define baudrate 115200

int32_t countTick1 = 0;
int32_t countTick2 = 0;
int32_t countTick3 = 0;
int valuesUpdated = 0;

void setup() {
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);
//  pinMode(encoder3A, INPUT);
//  pinMode(encoder3B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoder1A), computeRotation1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2A), computeRotation2, RISING);
//  attachInterrupt(digitalPinToInterrupt(encoder3A), computeRotation3, RISING);
  
  Serial.begin(baudrate);
  //while(Serial.available()) Serial.read();
}

void loop() {
  if (valuesUpdated){
    valuesUpdated = 0;
    writeToVex();
  }
  delay(1);
}

void computeRotation1() {
  if (digitalRead(encoder1B)) {
    countTick1 += 1;
  } else {
    countTick1 -= 1;
  }
  valuesUpdated = 1;
}

void computeRotation2() {
  if (digitalRead(encoder2B)) {
    countTick2 += 1;
  } else {
    countTick2 -= 1;
  }
  valuesUpdated = 1;
}

//void computeRotation3() {
//  if (digitalRead(encoder3B)) {
//    countTick3 += 1;
//  } else {
//    countTick3 -= 1;
//  }
//  valuesUpdated = 1;
//}

void writeToVex() {
  byte align = 0x80;
  Serial.write(&align, 1);
  Serial.write((uint8_t *)&countTick1, 4);
  Serial.write((uint8_t *)&countTick2, 4);
//  Serial.write((uint8_t *)&countTick3, 4);
}
