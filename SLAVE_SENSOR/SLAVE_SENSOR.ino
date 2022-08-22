const uint8_t amountOfSensor = 3;
const uint8_t echo[amountOfSensor]  = {PB4, PA15, PB14};
const uint8_t trig[amountOfSensor]  = {PB5, PB3, PB15};
int distance[amountOfSensor], duration;
uint8_t i;

void setup() {
  Serial.begin(115200);

  for (i = 0; i < amountOfSensor; i++) {
    pinMode(echo[i], INPUT);
    pinMode(trig[i], OUTPUT);
  }
}

void loop() {
  for (i = 0; i < amountOfSensor; i++) {
    digitalWrite(trig[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trig[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trig[i], LOW);

    duration = pulseIn(echo[i], HIGH, 10000);
    distance[i] = duration / 28 / 2;

    if (distance[i] == 0 || distance[i] > 200) {
      distance[i] = 200;
    }
    else if (distance[i] < 10) {
      distance[i] = 10;
    }
  }

//For Testing
//  for (i = 0; i < amountOfSensor; i++) {
//    Serial.print(distance[i]);
//    Serial.print(",");
//  }
//  Serial.println("#");

//For Robot
  char inChar = (char)Serial.read();
  if (inChar == '*') {
    for (i = 0; i < amountOfSensor; i++) {
      Serial.print(distance[i]);
      Serial.print(",");
    }
    Serial.println("#");
  }
}
