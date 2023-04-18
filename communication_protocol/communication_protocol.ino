#define RECEIVER    6
#define TRANSMITTER 7
#define FLAG        0x7e
#define ESCAPE      0x7d

void setup() {
  Serial.begin(9600);
  //pinMode(RECEIVER, INPUT);
  //pinMode(TRANSMITTER, OUTPUT);
}

void loop() {
  int length = 2;
  uint8_t *temp = (uint8_t*) malloc(length * sizeof(uint8_t));
  temp[0] = FLAG;
  temp[1] = ESCAPE;
  Serial.print("\nBEFORE:\n");
  for (int i = 0; i < length; ++i) {
    Serial.print(temp[i], HEX);
  }
  length = data_stuffing(temp, length);
  Serial.print("\nAFTER:\n");
  for (int i = 0; i < length; ++i) {
    Serial.print(temp[i], HEX);
  }
  free(temp);

  delay(1000);
}

void create_frame(uint8_t *frame, uint8_t *data, uint8_t length) {

}

uint8_t calculate_bcc(uint8_t *data, uint8_t length) {
  uint8_t bcc = 0;
  for (int i = 0; i < length; ++i) {
    bcc = data[i] ^ bcc;
  }
  return bcc;
}

uint8_t data_stuffing(uint8_t *data, uint8_t length) {

  // Calculating new length
  uint8_t new_length = length;
  for (int i = 0; i < length; ++i) {
    if (data[i] == FLAG || data[i] == ESCAPE) {
      new_length++;
    }
  }

  Serial.println(new_length);

  // Allocating new data
  uint8_t *new_data = (uint8_t*) malloc(new_length * sizeof(uint8_t));
  Serial.println("HERE");
  if (new_data == NULL) {
    Serial.println("Error on new_data malloc");
    return 0;
  }

  // Data stuffing
  int x = 0;
  for (int y = 0; y < length; ++y) {
    if (data[y] == FLAG || data[y] == ESCAPE) {
      new_data[x] = ESCAPE;
      new_data[x+1] = (uint8_t) (data[y] ^ 0x20);
      x += 2;
    } else {
      new_data[x] = data[y];
      x++;
    }
  }

  Serial.println((unsigned int) data);
  Serial.println((unsigned int) new_data);

  uint8_t *old_data = data;
  data = new_data;
  free(old_data);

  return new_length;
}

uint8_t data_destuffing(uint8_t *data, uint8_t length) {

}
