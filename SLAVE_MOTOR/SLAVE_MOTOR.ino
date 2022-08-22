//Importing all the necesarry library
#include "HardwareTimer.h"

//Set PWM frequencies to 20khz
#define freqPWM 20000
#define freqCPU (72000000/freqPWM)
#define maxOriginalPWM 65535
#define amountOfMotor 4
#define MAPfreq labs((CarrierPWM/maxOriginalPWM) * (freqCPU))
HardwareTimer pwmtimer(2);

//Parsing data variable
String dataIn;
String arrayData[8];
boolean parsing = false,
        receive = false;

//Motor variable
const uint8_t pwmPin[amountOfMotor]  = {PA3, PA2, PA0, PA1};
const uint8_t enaAPin[amountOfMotor] = {PB10, PB0, PA4, PA6};
const uint8_t enaBPin[amountOfMotor] = {PB11, PB1, PA5, PA7};
const uint8_t encAPin[amountOfMotor] = {PB8, PB7, PB14, PB12};
const uint8_t encBPin[amountOfMotor] = {PB9, PB6, PB15, PB13};
const short int encCount[amountOfMotor] = {1988, 1988, 1988, 1988};
int revolutions[amountOfMotor],
    setRPM[amountOfMotor],
    nowRPM[amountOfMotor];

//Timing Variable
unsigned int prevTime, nowTime;
const uint8_t interval = 100;

//PID variable
double PIDValue[amountOfMotor], PWMValue[amountOfMotor], CarrierPWM,
       Error[amountOfMotor], PrevError[amountOfMotor],
       P[amountOfMotor], I[amountOfMotor], D[amountOfMotor];
double Kp[amountOfMotor] = {100, 100, 100, 100};
double Ki[amountOfMotor] = {0, 0, 0, 0};
double Kd[amountOfMotor] = {0, 0, 0, 0};
int final_speed[amountOfMotor];

unsigned int i, maxRPM = 50, master_command = 0;

void setup() {
  Serial.begin(115200);

  for (i = 0; i < amountOfMotor; i++) {
    pinMode(pwmPin[i], PWM);
    pinMode(enaAPin[i], OUTPUT);
    pinMode(enaBPin[i], OUTPUT);
    pinMode(encAPin[i], INPUT);
    pinMode(encBPin[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(encAPin[0]), func1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAPin[1]), func2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAPin[2]), func3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAPin[3]), func4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPin[0]), func1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPin[1]), func2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPin[2]), func3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPin[3]), func4, CHANGE);

  pwmtimer.setPrescaleFactor(1);
  pwmtimer.setOverflow(freqCPU);

  prevTime = millis();
}

void loop() {
  readSerial();

  //  0 = 'Diagonal Kanan'
  //  1 = 'Diagonal Kiri'
  //  2 = 'Diam'
  //  3 = 'Kanan'
  //  4 = 'Kiri'
  //  5 = 'Maju'

  switch (master_command) {
    case 0:
      setRPM[0] = 0;
      setRPM[1] = -maxRPM;
      setRPM[2] = maxRPM;
      setRPM[3] = 0;
      break;
    case 1:
      setRPM[0] = 0;
      setRPM[1] = maxRPM;
      setRPM[2] = -maxRPM;
      setRPM[3] = 0;
      break;
    case 2:
      setRPM[0] = 0;
      setRPM[1] = 0;
      setRPM[2] = 0;
      setRPM[3] = 0;
      break;
    case 3:
      setRPM[0] = maxRPM;
      setRPM[1] = -maxRPM;
      setRPM[2] = maxRPM;
      setRPM[3] = -maxRPM;
      break;
    case 4:
      setRPM[0] = -maxRPM;
      setRPM[1] = maxRPM;
      setRPM[2] = -maxRPM;
      setRPM[3] = maxRPM;
      break;
    case 5:
      setRPM[0] = maxRPM;
      setRPM[1] = maxRPM;
      setRPM[2] = -maxRPM;
      setRPM[3] = -maxRPM;
      break;
    default:
      setRPM[0] = 0;
      setRPM[1] = 0;
      setRPM[2] = 0;
      setRPM[3] = 0;
      break;
  }

  nowTime = millis();
  if (nowTime - prevTime > interval) {
    prevTime = nowTime;

    // Calculate RPM
    for (i = 0; i < amountOfMotor; i++) {
      nowRPM[i] = (float)(revolutions[i] * 600 / encCount[i]);
    }

    PIDMotor();
  }

  for (i = 0; i < amountOfMotor; i++) {
    Serial.print(nowRPM[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void readSerial() {
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read();
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
  else if (!Serial) {
    master_command = 2;
    Serial.println("Serial Disconnected");
  }
}

void parsingData() {
  short int j = 0;
  arrayData[j] = "";

  for (i = 1; i < dataIn.length(); i++) {
    if ((dataIn[i] == '#') || (dataIn[i] == ',')) {
      j++;
      arrayData[j] = "";
    }
    else {
      arrayData[j] += dataIn[i];
    }
  }

  master_command = arrayData[0].toFloat();
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
    I[i] = PrevError[i] + Error[i];
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

    CarrierPWM = PWMValue[i];

    pwmWrite(pwmPin[i], MAPfreq);
    Serial.print(MAPfreq);
    Serial.print("\t");

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
  Serial.println();
}
