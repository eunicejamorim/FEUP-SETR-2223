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

#define B_SPEED_OFFSET 7

int right_motor = 0;
int left_motor = 0;

void move()
{
  analogWrite(PWMA, abs(left_motor));
  analogWrite(PWMB, abs(right_motor));
  digitalWrite(AIN1, left_motor >= 0 ? LOW : HIGH);
  digitalWrite(AIN2, left_motor > 0 ? HIGH : LOW);
  digitalWrite(BIN1, right_motor >= 0 ? LOW : HIGH);
  digitalWrite(BIN2, right_motor > 0 ? HIGH : LOW);
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
    right_motor = speed - B_SPEED_OFFSET;
    left_motor = speed;
    stopped = 0;
  } else if (stopped ? (distance < stopped_distance - tolerance) : (distance < minDist - tolerance)) {
    right_motor = -speed + B_SPEED_OFFSET;
    left_motor = -speed;
    stopped = 0;
  } else {
    right_motor = 0;
    left_motor = 0;
    stopped = 1;
    stopped_distance = distance;
  }

  move();

  display.clearDisplay();
  display.setCursor(26 - 3 * (distance < 10 ? 0 : distance < 100 ? 1 : 2), 0);
  display.print("Distance: "); display.print(distance); display.println("cm");
  display.setCursor(0, 16);
  display.print("Right motor: "); display.println(right_motor);
  display.setCursor(0, 32);
  display.print("Left motor: "); display.println(left_motor);
  display.display();
}
