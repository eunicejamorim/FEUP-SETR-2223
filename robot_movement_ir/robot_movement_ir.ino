#include <IRremote.hpp>
#include <Adafruit_SSD1306.h>

// DEFINES
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
int rotate_speed = 30;

unsigned int command;

void translateIR()
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

void move()
{
  analogWrite(PWMA, abs(left_motor));
  analogWrite(PWMB, abs(right_motor));
  digitalWrite(AIN1, left_motor >= 0 ? LOW : HIGH);
  digitalWrite(AIN2, left_motor > 0 ? HIGH : LOW);
  digitalWrite(BIN1, right_motor >= 0 ? LOW : HIGH);
  digitalWrite(BIN2, right_motor > 0 ? HIGH : LOW);
}

void decodeIR() {
  if (IrReceiver.decode()) {
      command = IrReceiver.decodedIRData.command;  // Print "old" raw data
      IrReceiver.resume();
  }
}

void displayData() {
  display.clearDisplay();
  display.setCursor(0, 16);
  display.print("Right motor: "); display.println(right_motor);
  display.setCursor(0, 32);
  display.print("Left motor: "); display.println(left_motor);
  display.display();
}


void setup() {
  Serial.begin(9600);
  Serial.println("Front Car Setup Started");

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

  Sched_Init();
  Sched_AddT(decodeIR, 1, 10);
  Sched_AddT(translateIR, 1, 10);
  Sched_AddT(move, 1, 10);
  Sched_AddT(displayData, 1, 10);

  Serial.println("Front Car Setup Completed");
}

void loop() {
  /*if (IrReceiver.decode()) {
    command = IrReceiver.decodedIRData.command;  // Print "old" raw data
    translateIR();
    move();
    IrReceiver.resume();  // Enable receiving of the next value
  }

  displayData();*/
}