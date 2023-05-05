#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
   
//wiring vin - 5v gnd - gnd slc-a5 sda-a4   

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

void configureSensor(void)
{
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

float error = 0;
float currentAngle = 0;
long int time;

float getYaw(){
  return currentAngle;
}

void updateError()
{
  int c = 200;
  for (int i = 0; i < c; i++) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp); 

    time = gyro.timestamp;
    error += gyro.gyro.z;
    delay(10);
  }

  error /= c;
}

/**************************************************************************/
void updateAngle(){
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  long int currentTime = gyro.timestamp;
  long int timeElapsed = currentTime-time;
  time = currentTime;
  currentAngle += 180 * (gyro.gyro.z - error) / M_PI * (timeElapsed) * 0.001;
}

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("LSM9DS0 9DOF Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("Found LSM9DS0 9DOF");
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
  updateError();
}

void loop(void) 
{  
  updateAngle();
  Serial.print(getYaw());
  Serial.print("\n");
  delay(10);
}