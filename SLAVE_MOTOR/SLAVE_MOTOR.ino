//Importing all the necesarry library
#include "HardwareTimer.h"

//Set PWM frequencies to 20khz
#define freqPWM 20000
#define freqCPU (72000000/freqPWM)
#define maxOriginalPWM 65535
#define amountOfMotor 4
#define MAPfreq1 labs((pwmValue[0]/maxOriginalPWM) * (freqCPU))
#define MAPfreq2 labs((pwmValue[1]/maxOriginalPWM) * (freqCPU))
#define MAPfreq3 labs((pwmValue[2]/maxOriginalPWM) * (freqCPU))
#define MAPfreq4 labs((pwmValue[3]/maxOriginalPWM) * (freqCPU))
HardwareTimer pwmtimer(amountOfMotor);

//Parsing data variable
String dataIn;
String arrayData[8];
boolean parsing = false,
        receive = false,
        readMasterState = true;

//Motor variable
const uint8_t pwmPin[amountOfMotor]  = {PA0, PA1, PA2, PA3};
const uint8_t enaAPin[amountOfMotor] = {PA4, PA6, PB0, PB10};
const uint8_t enaBPin[amountOfMotor] = {PA5, PA7, PB1, PB11};
const uint8_t encAPin[amountOfMotor] = {PB8, PB7, PB14, PB12};
const uint8_t encBPin[amountOfMotor] = {PB9, PB6, PB15, PB13};
const short int encCount[amountOfMotor] = {1988, 1988, 1988, 1988};
float revolutions[amountOfMotor],
      setRPM[amountOfMotor],
      nowRPM[amountOfMotor];

//Relay variable
const uint8_t amountOfRelay = 4;
const uint8_t relayPin[amountOfRelay] = {PA15, PB3, PB4, PB5};
uint8_t relayState[amountOfRelay];

//Timing Variable
unsigned int prevTime, nowTime;
const uint8_t interval = 100;

//PID variable

double PIDValue[amountOfMotor], PWMValue[amountOfMotor],
       Error[amountOfMotor], PrevError[amountOfMotor],
       P[amountOfMotor], I[amountOfMotor], D[amountOfMotor];
double Kp[amountOfMotor] = {0, 0, 0, 0};
double Ki[amountOfMotor] = {0, 0, 0, 0};
double Kd[amountOfMotor] = {0, 0, 0, 0};

uint8_t i;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  for (i = 0; i < amountOfMotor; i++) {
    pinMode(pwmPin[i], PWM);
    pinMode(enaAPin[i], OUTPUT);
    pinMode(enaBPin[i], OUTPUT);
    pinMode(encAPin[i], INPUT);
    pinMode(encBPin[i], INPUT);
  }

  for (i = 0; i < amountOfRelay; i++) {
    pinMode(relayPin[i], OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(encAPin[0]), func1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAPin[1]), func2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAPin[2]), func3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAPin[3]), func4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPin[0]), func1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPin[1]), func2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPin[2]), func3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPin[3]), func4, CHANGE);

  prevTime = millis();
}

void loop() {
  readSerial();
  readMaster();

  nowTime = millis();
  if (nowTime - prevTime > interval) {
    prevTime = nowTime;

    // Calculate RPM
    for (i = 0; i < amountOfMotor; i++) {
      nowRPM[i] = (float)(revolutions[i] * 600 / encCount[i]);
    }

    PIDMotor();
  }
}

void readMaster() {
  while (Serial1.available() > 0 && readMasterState == true) {
    char inChar = (char)Serial1.read();
    if (inChar == '*') {
      dataIn = "";
      receive = true;
    }

    if (receive) {
      dataIn += inChar;
      if (inChar == '#') {
        parsing = true;
      }
    }

    if (parsing) {
      parsingData();
      parsing = false;
      receive = false;
    }
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '*') {
      dataIn = "";
      receive = true;
    }

    else if (inChar == 'E') {
      readMasterState = !readMasterState;
    }

    if (receive) {
      dataIn += inChar;
      if (inChar == '#') {
        parsing = true;
      }
    }

    if (parsing) {
      parsingData();
      parsing = false;
      receive = false;
    }
  }
}

void parsingData() {
  short int j = 0;
  arrayData[j] = "";

  for (int i = 1; i < dataIn.length(); i++) {
    if ((dataIn[i] == '#') || (dataIn[i] == ',')) {
      j++;
      arrayData[j] = "";
    }
    else {
      arrayData[j] += dataIn[i];
    }
  }

  setRPM[0] = arrayData[0].toFloat();
  setRPM[1] = arrayData[1].toFloat();
  setRPM[2] = arrayData[2].toFloat();
  setRPM[3] = arrayData[3].toFloat();

  relayState[0]  = arrayData[4].toFloat();
  relayState[1]  = arrayData[5].toFloat();
  relayState[2]  = arrayData[6].toFloat();
  relayState[3]  = arrayData[7].toFloat();
}

void func1() {
  revolutions[0]++;;
}

void func2() {
  revolutions[1]++;;
}

void func3() {
  revolutions[2]++;;
}

void func4() {
  revolutions[3]++;;
}

void PIDMotor() {
  for (i = 0; i < amountOfMotor; i++) {
    Error[i] = abs(setRPM[i]) - nowRPM[i];
    P[i] = Error[i];
    I[i] = I[i] + Error[i];
    D[i] = Error[i] - PrevError[i];

    if (I[i] >= 65535) {
      I[i] = 65535;
    }
    else if (I[i] <= 0) {
      I[i] = 0;
    }

    PIDValue[i] = (Kp[i] * P[i]) + (Ki[i] * I[i]) + (Kd[i] * D[i]);
    PrevError[i] = Error[i];
    PWMValue[i] = PWMValue[i] + PIDValue[i];

    if (PWMValue[i] >= 65535) {
      PWMValue[i] = 65535;
    }
    else if (PWMValue[i] <= 0) {
      PWMValue[i] = 0;
    }

    pwmWrite(pwmPin[i], PWMValue[i]);

    if (setRPM[i] > 0) {
      digitalWrite(enaAPin[i], HIGH);
      digitalWrite(enaBPin[i], LOW);
    }
    else if (setRPM[i] < 0) {
      digitalWrite(enaAPin[i], LOW);
      digitalWrite(enaBPin[i], HIGH);
    }
    else {
      digitalWrite(enaAPin[i], LOW);
      digitalWrite(enaBPin[i], LOW);
    }

    revolutions[i] = 0;
  }
}
