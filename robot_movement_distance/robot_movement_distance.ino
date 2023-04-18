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

#define ECHO 4
#define TRIG 5

#define B_SPEED_OFFSET 6

void forward(int speed) {
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed - B_SPEED_OFFSET);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void backwards(int speed) {
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed - B_SPEED_OFFSET);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void stop() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

int getDistance()  // Measure the distance
{
  digitalWrite(TRIG, LOW);  // set trig pin low 2μs
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);  // set trig pin 10μs , at last 10us
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);                // set trig pin low
  float Fdistance = pulseIn(ECHO, HIGH);  // Read echo pin high level time(us)
  Fdistance = Fdistance / 58;             //Y m=（X s*344）/2
  // X s=（ 2*Y m）/344 ==》X s=0.0058*Y m ==》cm = us /58
  return (int)Fdistance;
}

void setup() {
  // put your setup code here, to run once:
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.setTextSize(1);
  display.setTextColor(WHITE);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

int speed = 100;

int minDist = 30;   // min distance to target in cm
int tolerance = 2;  // tolerance for the distance in cm

int stopped = 0;
int stopped_distance = 0;

void loop() {
  // put your main code here, to run repeatedly:

  int distance = getDistance();

  if (stopped ? (distance > stopped_distance + tolerance) : (distance > minDist + tolerance)) {
    forward(speed);
    stopped = 0;
  } else if (stopped ? (distance < stopped_distance - tolerance) : (distance < minDist - tolerance)) {
    backwards(speed);
    stopped = 0;
  } else {
    stop();
    stopped = 1;
    stopped_distance = distance;
  }

  display.clearDisplay();
  display.setCursor(26 - 3 * (distance < 10 ? 0 : distance < 100 ? 1 : 2), 0);
  display.print("Distance: "); display.print(distance); display.println("cm");
  display.display();
}
