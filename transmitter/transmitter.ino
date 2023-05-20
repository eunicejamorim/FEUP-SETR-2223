#define CPU_FREQ 16000000L  // cpu clock
#define PRESCALER 256       // cpu prescaler

const unsigned long TICKS_PER_MILISEC = ( CPU_FREQ / PRESCALER ) / 1000;
const unsigned short INTERRUPT_INTERVAL = 1;                                        // interrupt every X miliseconds
const unsigned short INTERRUPT_TICKS = INTERRUPT_INTERVAL * TICKS_PER_MILISEC - 1;  // amount of ticks in an interrupt intervall

void setup() {
    Serial.begin(9600);

    noInterrupts();
    TCNT1 = 0;                  // delete timer counter register
    TCCR1A = 0;                 // delete TCCR1A-Registers
    TCCR1B = 0;                 // delete TCCR1B-Registers
    TCCR1B |= (1 << WGM12);     // CTC-Mode (Waveform Generation Mode): resets TCNT1 to0 after interrupt, makes OCR1A the leading compare register
    TCCR1B |= (1 << CS12);      // set prescaler to 256
    TIMSK1 |= (1 << OCIE1A);    // enable interrupts
    OCR1A = INTERRUPT_TICKS;
    interrupts();  
}

void loop() {
    send_byte(0b01010011);
}

int timer = 0;
int bit = 0;
bool sending = false;
uint8_t value;

ISR(TIMER1_COMPA_vect) {
    timer++;
    if (sending && timer >= 100) {
        timer = 0;
        digitalWrite(2, value % 2);
        value >>= 1;
        bit++;
        if (bit == 8) {
            sending = false;
        }
    }
}

void send_byte(uint8_t byte) {
    if (!sending) {
        sending = true;
        value = byte;
        timer = 0;
        bit = 0;
        digitalWrite(2, LOW);
    }
}
