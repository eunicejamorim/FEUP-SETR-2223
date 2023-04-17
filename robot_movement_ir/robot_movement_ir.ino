#include <IRremote.hpp>

#define PWMA 10  //Left Motor Speed pin (ENA)
#define AIN2 A0  //Motor-L forward (IN2).
#define AIN1 A1  //Motor-L backward (IN1)
#define PWMB 11  //Right Motor Speed pin (ENB)
#define BIN1 A2  //Motor-R forward (IN3)
#define BIN2 A3  //Motor-R backward (IN4)
#define IR 7

#define KEY2 0x18  //Key:2
#define KEY8 0x52  //Key:8
#define KEY4 0x08  //Key:4
#define KEY6 0x5A  //Key:6
#define KEY5 0x1C  //Key:5

void translateIR(unsigned int command)  // takes action based on IR code received
// describing KEYES Remote IR codes
{
  switch (command) {
    case KEY2:
      forward();
      break;
    case KEY4:
      left();
      break;
    case KEY5:
      stop();
      break;
    case KEY6:
      right();
      break;
    case KEY8:
      backward();
      break;
    default:
      stop();
      break;
  }
}

void forward() {
  analogWrite(PWMA, 100);
  analogWrite(PWMB, 100);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void backward() {
  analogWrite(PWMA, 100);
  analogWrite(PWMB, 100);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void right() {
  analogWrite(PWMA, 30);
  analogWrite(PWMB, 30);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void left() {
  analogWrite(PWMA, 30);
  analogWrite(PWMB, 30);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void stop() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


void setup() {
  IrReceiver.begin(IR, ENABLE_LED_FEEDBACK);  // Start the receiver
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void loop() {
  if (IrReceiver.decode()) {
    unsigned int command = IrReceiver.decodedIRData.command;  // Print "old" raw data
    translateIR(command);
    IrReceiver.resume();  // Enable receiving of the next value
  }
}