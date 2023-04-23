#include <IRremote.hpp>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 9
#define OLED_SA0   8
Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);

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

int right_motor = 0;
int left_motor = 0;

int speed = 100;
int rotate_speed = 30;

void move()
{
  analogWrite(PWMA, abs(left_motor));
  analogWrite(PWMB, abs(right_motor));
  digitalWrite(AIN1, left_motor >= 0 ? LOW : HIGH);
  digitalWrite(AIN2, left_motor > 0 ? HIGH : LOW);
  digitalWrite(BIN1, right_motor >= 0 ? LOW : HIGH);
  digitalWrite(BIN2, right_motor > 0 ? HIGH : LOW);
}

void translateIR(unsigned int command)  // takes action based on IR code received
// describing KEYES Remote IR codes
{
  switch (command) {
    case KEY2:
      right_motor = speed;
      left_motor = speed;
      break;
    case KEY4:
      right_motor = rotate_speed;
      left_motor = -rotate_speed;
      break;
    case KEY5:
      right_motor = 0;
      left_motor = 0;
      break;
    case KEY6:
      right_motor = -rotate_speed;
      left_motor = rotate_speed;
      break;
    case KEY8:
      right_motor = -speed;
      left_motor = -speed;
      break;
    default:
      right_motor = 0;
      left_motor = 0;
      break;
  }
}


void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.setTextSize(1);
  display.setTextColor(WHITE);

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
    move();
    IrReceiver.resume();  // Enable receiving of the next value
  }

  display.clearDisplay();
  display.setCursor(0, 16);
  display.print("Right motor: "); display.println(right_motor);
  display.setCursor(0, 32);
  display.print("Left motor: "); display.println(left_motor);
  display.display();
}