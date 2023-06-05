#define DECODE_SAMSUNG
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.hpp>

// DEFINES
#define OLED_RESET 9
#define OLED_SA0 8
Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);

Adafruit_LSM9DS0 lsm(1000); // Use I2C, ID #1000

#define IR_TRANSMITTER 12

#define CPU_FREQ 16000000L   // cpu clock
#define PRESCALER 256        // cpu prescaler
#define BAUD_RATE 10         // comms baud rate
#define INTERRUPT_INTERVAL 1 // interrupt every X miliseconds

const unsigned long TICKS_PER_MILISEC = (CPU_FREQ / PRESCALER) / 1000;
const unsigned short TICKS_PER_INTERRUPT = INTERRUPT_INTERVAL * TICKS_PER_MILISEC - 1; // amount of ticks in an interrupt interval

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

int Sched_Init(void) {
    for (int x = 0; x < NT; x++)
        Tasks[x].func = 0;

    noInterrupts();
    TCNT1 = 0;               // delete timer counter register
    TCCR1A = 0;              // delete TCCR1A-Registers
    TCCR1B = 0;              // delete TCCR1B-Registers
    TCCR1B |= (1 << WGM12);  // CTC-Mode (Waveform Generation Mode): resets TCNT1 to0 after interrupt, makes OCR1A the leading compare register
    TCCR1B |= (1 << CS12);   // set prescaler to 256
    TIMSK1 |= (1 << OCIE1A); // enable interrupts
    OCR1A = TICKS_PER_INTERRUPT;
    interrupts();
}

int Sched_AddT(void (*f)(void), int d, int p) {
    for (int x = 0; x < NT; x++)
        if (!Tasks[x].func) {
            Tasks[x].period = p;
            Tasks[x].delay = d;
            Tasks[x].exec = 0;
            Tasks[x].func = f;
            return x;
        }
    return -1;
}

void Sched_Schedule(void) {
    for (int x = 0; x < NT; x++) {
        if (Tasks[x].func) {
            if (Tasks[x].delay) {
                Tasks[x].delay--;
            } else {
                /* Schedule Task */
                Tasks[x].exec++;
                Tasks[x].delay = Tasks[x].period - 1;
            }
        }
    }
}

void Sched_Dispatch(void) {
    int prev_task = cur_task;
    for (int x = 0; x < cur_task; x++) {
        if ((Tasks[x].func) && (Tasks[x].exec)) {
            Tasks[x].exec = 0;
            cur_task = x;
            interrupts();
            Tasks[x].func();
            noInterrupts();
            cur_task = prev_task;
            /* Delete task if one-shot */
            if (!Tasks[x].period)
                Tasks[x].func = 0;
        }
    }
}

ISR(TIMER1_COMPA_vect) { // timer1 interrupt
    Sched_Schedule();
    Sched_Dispatch();
}

float angleError = 0;
float currentAngle = 0;
long int angleTime;

void commsIR() {
    int16_t angleSend = currentAngle * 1000.0f;
    IrSender.sendSamsung(0x69, angleSend, 0);
}

void configureAngleSensor(void) { lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS); }

void updateAngleError() {
    int c = 200;
    sensors_event_t gyro;
    for (int i = 0; i < c; i++) {
        delay(20);
        lsm.getGyro().getEvent(&gyro);

        angleTime = gyro.timestamp;
        angleError += gyro.gyro.z;
    }

    angleError /= c;
}

void updateAngle() {
    sensors_event_t gyro;
    lsm.getGyro().getEvent(&gyro);
    long int currentTime = gyro.timestamp;
    long int timeElapsed = currentTime - angleTime;
    angleTime = currentTime;
    currentAngle += (gyro.gyro.z - angleError) * timeElapsed * 0.001;
}

void displayData() {
    display.clearDisplay();
    display.setCursor(25 - 3 * (abs(currentAngle) < 10 ? 0 : abs(currentAngle) < 100 ? 1 : 2) - (currentAngle < 0 ? 3 : 0), 0);
    display.print(F("Angle: "));
    display.print(180 * currentAngle / M_PI);
    display.print(F("dg"));
    display.display();
}

void setup() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.clearDisplay();
    display.setCursor(1, 0);
    display.print(F("Calibration in 1000ms"));
    display.setCursor(30, 8);
    display.print(F("Stand Still"));
    display.display();

    delay(1000);
    lsm.begin();
    configureAngleSensor();
    updateAngleError();

    IrSender.begin(IR_TRANSMITTER);

    Sched_Init();
    //Sched_AddT(decodeIR, 1, 10);
    //Sched_AddT(translateCommands, 1, 50);
    //Sched_AddT(move, 1, 50);
    Sched_AddT(displayData, 1, 500);
    Sched_AddT(commsIR, 1, 200);
    Sched_AddT(updateAngle, 1, 10);
}

void loop() {
    // NOTHING TO DO
}