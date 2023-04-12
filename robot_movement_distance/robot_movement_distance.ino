
#define PWMA   10           //Left Motor Speed pin (ENA)
#define AIN2   A0          //Motor-L forward (IN2).
#define AIN1   A1          //Motor-L backward (IN1)

#define PWMB   11           //Right Motor Speed pin (ENB)
#define BIN1   A2          //Motor-R forward (IN3)
#define BIN2   A3          //Motor-R backward (IN4)

#define ECHO   4
#define TRIG   5
  
#define SPEED_B_OFFSET 7

int movingForward = 0;
int movingBackwards = 0;

void forward(int speed)
{
  analogWrite(PWMA,speed);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  analogWrite(PWMB,speed-SPEED_B_OFFSET);
  digitalWrite(BIN1,LOW); 
  digitalWrite(BIN2,HIGH);
  movingForward = 1;
  movingBackwards = 0;
}

void backwards(int speed)
{
  analogWrite(PWMA,speed);
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  analogWrite(PWMB,speed-SPEED_B_OFFSET);
  digitalWrite(BIN1,HIGH); 
  digitalWrite(BIN2,LOW);
  movingBackwards = 1;
  movingForward = 0;
}

void stop()
{
  analogWrite(PWMA,0);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  analogWrite(PWMB,0);
  digitalWrite(BIN1,LOW); 
  digitalWrite(BIN2,LOW);
  movingBackwards = 0;
  movingForward = 0;
}

int getDistance()         // Measure the distance 
{
  digitalWrite(TRIG, LOW);   // set trig pin low 2μs
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);  // set trig pin 10μs , at last 10us 
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);    // set trig pin low
  float Fdistance = pulseIn(ECHO, HIGH);  // Read echo pin high level time(us)
  Fdistance= Fdistance/58;       //Y m=（X s*344）/2
  // X s=（ 2*Y m）/344 ==》X s=0.0058*Y m ==》cm = us /58       
  return (int)Fdistance;
}  

void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA,OUTPUT);                     
  pinMode(AIN2,OUTPUT);      
  pinMode(AIN1,OUTPUT);
  pinMode(PWMB,OUTPUT);       
  pinMode(AIN1,OUTPUT);     
  pinMode(AIN2,OUTPUT);  
}

int speed = 80;

int minDist = 20; // min distance to target in cm
int tolerance = 2; // tolerance for the distance in cm

void loop() {
  // put your main code here, to run repeatedly:
  
  int distance = getDistance();

  if (!movingForward && distance >= minDist + tolerance)
  {
    forward(50);
  }
  else if (!movingBackwards && distance <= minDist - tolerance)
  {
    backwards(50);
  }
  else if ((movingForward || movingBackwards) && distance >= minDist - tolerance && distance <= minDist + tolerance)
  {
    stop();
  }
}
