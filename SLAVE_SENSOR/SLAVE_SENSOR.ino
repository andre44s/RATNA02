const uint8_t amountOfSensor = 3;
const uint8_t echo[amountOfSensor]  = {PB4, PA15, PB15};
const uint8_t trig[amountOfSensor]  = {PB5, PB3, PB14};
int distance[amountOfSensor], duration;
uint8_t i;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);

  for (i = 0; i < amountOfSensor; i++) {
    pinMode(echo[i], INPUT);
    pinMode(trig[i], OUTPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  for (i = 0; i < amountOfSensor; i++) {
    digitalWrite(trig[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trig[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trig[i], LOW);
    
    duration = pulseIn(echo[i], HIGH, 10000);
    distance[i] = duration / 28 / 2 ;

    Serial.println(distance[i]);
  }  
}
