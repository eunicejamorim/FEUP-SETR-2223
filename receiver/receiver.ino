#define CPU_FREQ 16000000L      // cpu clock
#define PRESCALER 256           // cpu prescaler
#define BAUD_RATE 10            // comms baud rate
#define INTERRUPT_INTERVAL 1    // interrupt every X miliseconds

const unsigned long TICKS_PER_MILISEC = ( CPU_FREQ / PRESCALER ) / 1000;
const unsigned short TICKS_PER_INTERRUPT = INTERRUPT_INTERVAL * TICKS_PER_MILISEC - 1;  // amount of ticks in an interrupt interval
const unsigned short INTERRUPTS_PER_BIT = 1000 / (BAUD_RATE * INTERRUPT_INTERVAL);

volatile enum CommsState { IDLE, INIT_BIT, BYTE, FINAL_BIT } state = IDLE;
uint8_t buffer = 0;

ISR(TIMER1_COMPA_vect) {
    static int interrupt_count = 0;
    static int bits_received = 0;

    switch (state) {
        case IDLE:
            break;
        case INIT_BIT:
            interrupt_count = 0;
            if (digitalRead(2) == LOW) {
                buffer = 0;
                bits_received = 0;
                state = BYTE;
            } else {
                Serial.println("> COMMS ERROR: Initial bit not LOW");
                state = IDLE;
                attachInterrupt(digitalPinToInterrupt(2), start_receiving, FALLING);
            }
            break;
        case BYTE:
            interrupt_count++;
            if (interrupt_count >= INTERRUPTS_PER_BIT - 1) {
                interrupt_count = 0;
                buffer |= digitalRead(2) << bits_received;
                bits_received++;
                if (bits_received == 8) {
                    state = FINAL_BIT;
                }
            }
            break;
        case FINAL_BIT:
            interrupt_count++;
            if (interrupt_count >= INTERRUPTS_PER_BIT - 1) {
                interrupt_count = 0;
                if (digitalRead(2) == HIGH) {
                    Serial.println(buffer);
                } else {
                    Serial.println("> COMMS ERROR: Final bit not HIGH");
                }
                state = IDLE;
                attachInterrupt(digitalPinToInterrupt(2), start_receiving, FALLING);
            }
            break;
    }
}

void start_receiving() {
    Serial.println(digitalRead(2));
    detachInterrupt(digitalPinToInterrupt(2));
    state = INIT_BIT;
}

void setup() {
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(2), start_receiving, FALLING); 

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

void loop() {}
