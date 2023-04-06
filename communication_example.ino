#define D6 6 //Arduino
#define D7 7 //Arduino

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(D6, INPUT);
  pinMode(D7, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(D7, HIGH);
  delay(10);
  if (digitalRead(D6)) { 
        Serial.println("MEGA UP");
  }
  delay(1000);
}
