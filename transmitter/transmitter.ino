#define CPU_FREQ 16000000L      // cpu clock
#define PRESCALER 256           // cpu prescaler
#define BAUD_RATE 10            // comms baud rate
#define INTERRUPT_INTERVAL 1    // interrupt every X miliseconds

const unsigned long TICKS_PER_MILISEC = ( CPU_FREQ / PRESCALER ) / 1000;
const unsigned short TICKS_PER_INTERRUPT = INTERRUPT_INTERVAL * TICKS_PER_MILISEC - 1;  // amount of ticks in an interrupt interval
const unsigned short INTERRUPTS_PER_BIT = 1000 / (BAUD_RATE * INTERRUPT_INTERVAL);

enum CommsState { IDLE, INIT_BIT, BYTE, FINAL_BIT } state = IDLE;
uint8_t buffer = 0;

ISR(TIMER1_COMPA_vect) {
    static int interrupt_count = 0;
    static int bits_sent = 0;

    switch (state) {
        case IDLE:
            digitalWrite(2, HIGH);
            break;
        case INIT_BIT:
            digitalWrite(2, LOW);
            interrupt_count++;
            if (interrupt_count >= INTERRUPTS_PER_BIT - 1) {
                interrupt_count = 0;
                bits_sent = 0;
                state = BYTE;
            }
            break;
        case BYTE:
            digitalWrite(2, buffer % 2);
            interrupt_count++;
            if (interrupt_count >= INTERRUPTS_PER_BIT - 1) {
                interrupt_count = 0;
                bits_sent++;
                if (bits_sent < 8) {
                    buffer /= 2;
                } else {
                    state = FINAL_BIT;
                }
            }
            break;
        case FINAL_BIT:
            digitalWrite(2, HIGH);
            interrupt_count++;
            if (interrupt_count >= (INTERRUPTS_PER_BIT - 1) * 4) {
                interrupt_count = 0;
                state = IDLE;
            }
            break;
    }
}

void send_byte(signed char byte) {
    if (state == IDLE) {
        state = INIT_BIT;
        buffer = byte;
    }
}

void setup() {
    Serial.begin(9600);

    noInterrupts();
    TCNT1 = 0;                  // delete timer counter register
    TCCR1A = 0;                 // delete TCCR1A-Registers
    TCCR1B = 0;                 // delete TCCR1B-Registers
    TCCR1B |= (1 << WGM12);     // CTC-Mode (Waveform Generation Mode): resets TCNT1 to0 after interrupt, makes OCR1A the leading compare register
    TCCR1B |= (1 << CS12);      // set prescaler to 256
    TIMSK1 |= (1 << OCIE1A);    // enable interrupts
    OCR1A = TICKS_PER_INTERRUPT;
    interrupts();
}

void loop() {
    delay(1);
    send_byte(-58);
}
