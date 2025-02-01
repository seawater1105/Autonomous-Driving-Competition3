#include <RBD_LightSensor.h>

RBD::LightSensor light_sensor(A0);

const int numReadings = 100;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

const int pinRight = 11;
const int pinLeft = 12;
const int pinCenter = 13;

int standardLight = 0;
int leftLight = 0;
int rightLight = 0;
int minLight = 0;
int l2 = 0;

int a = '0';
int b = '0';
//int c = '0';

void setup() {
  Serial.begin(9600);
  pinMode(pinRight, OUTPUT);
  pinMode(pinLeft, OUTPUT);
  pinMode(pinCenter, OUTPUT);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {

  total = total - readings[readIndex];
  readings[readIndex] = analogRead(A0);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {readIndex = 0;}

  int avg = total / numReadings;

  if (avg >= 0 && avg < 2) { standardLight = 255; }
  else if (avg >= 2 && avg < 5) { standardLight = 180; }
  else if (avg >= 5 && avg < 10) { standardLight = 100; }
  else if (avg >= 10) { standardLight = 0; }

  if (Serial.available() > 0) {
    int c = Serial.read();

    if (c != b) { // c와 a가 다른 경우에만
      a = b; // 현재 값에서 이전 값 빼기
      b = c;
    }

    // left -> center (b - a == 1)
    if (b - a == 1) {
      analogWrite(pinRight, rightLight);
      analogWrite(pinCenter, standardLight);
      
      while(leftLight>0){
        leftLight -= 1;
        analogWrite(pinLeft, leftLight);
      }
    }
    //right -> center (b - a == 2)
    else if (b - a == 2) {
      analogWrite(pinLeft, leftLight);
      analogWrite(pinCenter, standardLight);

      while(rightLight>0){
        rightLight -= 1;
        analogWrite(pinRight, rightLight);
      }
    }
    // center -> left (b - a == -1)
    if (b - a == -1) {
      analogWrite(pinRight, rightLight);
      analogWrite(pinLeft, min(leftLight += 5, standardLight));  // leftLight를 늘리고 최대값을 보장
      analogWrite(pinCenter, standardLight);  // center를 끄거나 낮은 값으로 설정
    }
    // center -> right (b - a == -2)
    else if (b - a == -2) {
      analogWrite(pinRight, min(rightLight += 5, standardLight));  // rightLight를 늘리고 최대값을 보장
      analogWrite(pinLeft, leftLight);
      analogWrite(pinCenter, standardLight);
    }
    // 기본 상태
    else {
      analogWrite(pinRight, rightLight);
      analogWrite(pinLeft, leftLight);
      analogWrite(pinCenter, standardLight);
    }
  }
}