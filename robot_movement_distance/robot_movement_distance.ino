#include <Adafruit_SSD1306.h>
#include <Adafruit_LSM9DS0.h>

//DEFINES
#define OLED_RESET 9
#define OLED_SA0   8
Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);

Adafruit_LSM9DS0 lsm(1000);

#define PWMA 6  //Left Motor Speed pin (ENA)
#define AIN2 A0  //Motor-L forward (IN2).
#define AIN1 A1  //Motor-L backward (IN1)

#define PWMB 11  //Right Motor Speed pin (ENB)
#define BIN1 A2  //Motor-R forward (IN3)
#define BIN2 A3  //Motor-R backward (IN4)

#define ECHO 4
#define TRIG 5

#define B_SPEED_OFFSET 7

// SHCEDULER 
int Sched_AddT(void (*f)(void), int d, int p);

typedef struct {
  /* period in ticks */
  int period;
  /* ticks until next activation */
  int delay;
  /* function pointer */
  void (*func)(void);
  /* activation counter */
  int exec;
} Sched_Task_t;

#define NT 20
Sched_Task_t Tasks[NT];
int cur_task = NT;

int Sched_Init(void){
  for(int x=0; x<NT; x++)
    Tasks[x].func = 0;
  /* Also configures interrupt that periodically calls Sched_Schedule(). */
  noInterrupts(); // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
 
//OCR1A = 6250; // compare match register 16MHz/256/10Hz
//OCR1A = 31250; // compare match register 16MHz/256/2Hz
  OCR1A = 31;    // compare match register 16MHz/256/2kHz
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS12); // 256 prescaler
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  interrupts(); // enable all interrupts  
}

int Sched_AddT(void (*f)(void), int d, int p){
  for(int x=0; x<NT; x++)
    if (!Tasks[x].func) {
      Tasks[x].period = p;
      Tasks[x].delay = d;
      Tasks[x].exec = 0;
      Tasks[x].func = f;
      return x;
    }
  return -1;
}


void Sched_Schedule(void){
  for(int x=0; x<NT; x++) {
    if(Tasks[x].func){
      if(Tasks[x].delay){
        Tasks[x].delay--;
      } else {
        /* Schedule Task */
        Tasks[x].exec++;
        Tasks[x].delay = Tasks[x].period-1;
      }
    }
  }
}


void Sched_Dispatch(void){
  int prev_task = cur_task;
  for(int x=0; x<cur_task; x++) {
    if((Tasks[x].func)&&(Tasks[x].exec)) {
      Tasks[x].exec=0;
      cur_task = x;
      interrupts();
      Tasks[x].func();
      noInterrupts();
      cur_task = prev_task;
      /* Delete task if one-shot */
      if(!Tasks[x].period) Tasks[x].func = 0;
    }
  }
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt
  Sched_Schedule();
  Sched_Dispatch();
}


// CODE
int right_motor = 0;
int left_motor = 0;

int speed = 100;

int minDist = 30;   // min distance to target in cm
int tolerance = 2;  // tolerance for the distance in cm

int stopped = 0;
int stopped_distance = 0;

int distance;

float angleError = 0;
float currentAngle = 0;
long int angleTime;

void configureAngleSensor(void)
{
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void updateAngleError()
{
  int c = 200;
  for (int i = 0; i < c; i++) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp); 

    angleTime = gyro.timestamp;
    angleError += gyro.gyro.z;
    delay(10);
  }

  angleError /= c;
}

void updateAngle(){
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  long int currentTime = gyro.timestamp;
  long int timeElapsed = currentTime-angleTime;
  angleTime = currentTime;
  currentAngle += 180 * (gyro.gyro.z - angleError) / M_PI * (timeElapsed) * 0.001;
}

void move()
{
  analogWrite(PWMA, abs(left_motor));
  analogWrite(PWMB, abs(right_motor));
  digitalWrite(AIN1, left_motor >= 0 ? LOW : HIGH);
  digitalWrite(AIN2, left_motor > 0 ? HIGH : LOW);
  digitalWrite(BIN1, right_motor >= 0 ? LOW : HIGH);
  digitalWrite(BIN2, right_motor > 0 ? HIGH : LOW);
}

void getDistance()  // Measure the distance
{
  digitalWrite(TRIG, LOW);  // set trig pin low 2μs
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);  // set trig pin 10μs , at last 10us
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);                // set trig pin low
  float Fdistance = pulseIn(ECHO, HIGH);  // Read echo pin high level time(us)
  Fdistance = Fdistance / 58;             //Y m=（X s*344）/2
  // X s=（ 2*Y m）/344 ==》X s=0.0058*Y m ==》cm = us /58
  distance = Fdistance;
}

void processDistance()
{
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
}

void displayData() {
  display.clearDisplay();
  display.setCursor(10, 0);
  display.print("D: "); display.print(distance); display.print("cm A: "); display.print(currentAngle); display.print("dg");
  display.setCursor(0, 16);
  display.print("Right motor: "); display.print(right_motor);
  display.setCursor(0, 32);
  display.print("Left motor: "); display.print(left_motor);
  display.display();
}

void setup() {
  lsm.begin();
  
  configureAngleSensor();

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

  updateAngleError();

  Sched_Init();
  Sched_AddT(getDistance, 1, 100);
  Sched_AddT(processDistance, 1, 100);
  Sched_AddT(move, 1, 100);
  Sched_AddT(displayData, 1, 100);
  Sched_AddT(updateAngle, 1, 10);
}

void loop() {
  // NOTHING TO DO
}
