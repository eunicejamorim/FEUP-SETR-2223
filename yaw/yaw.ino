#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
   
//wiring vin - 5v gnd - gnd slc-a5 sda-a4   

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

void configureSensor(void)
{
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

float currentAngle = 0;
long int time;

void setup(void) 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
  time = millis();
}
/**************************************************************************/
void updateAngle(){
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  long int currentTime = millis();
  long int timeElapsed = currentTime-time;
  time = currentTime;
  currentAngle += 180 * gyro.gyro.z / M_PI * (timeElapsed) * 0.001;
}

void loop(void) 
{  
  updateAngle();
  Serial.print(currentAngle);
  Serial.print("\n");
  delay(250);
}