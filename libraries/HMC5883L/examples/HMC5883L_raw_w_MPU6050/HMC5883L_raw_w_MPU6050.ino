#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"


MPU6050 accelgyro;
HMC5883L mag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

#define LED_PIN 13
bool blinkState = false;

void setup() {
  
   Wire.begin();
   accelgyro.setI2CMasterModeEnabled(false);
   accelgyro.setI2CBypassEnabled(true) ;
   accelgyro.setSleepEnabled(false);

   Serial.begin(115200);

   // initialize device
   Serial.println("Initializing I2C devices...");
   accelgyro.initialize();
   mag.initialize();
   Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

   // verify connection
   Serial.println("Testing device connections...");
   Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

   // configure Arduino LED for
   pinMode(LED_PIN, OUTPUT);
}

void loop() {
   
   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   mag.getHeading(&mx, &my, &mz);

   // display tab-separated accel/gyro x/y/z values
   Serial.print("a/g:\t");
   Serial.print(ax); Serial.print("\t");
   Serial.print(ay); Serial.print("\t");
   Serial.print(az); Serial.print("\t");
   Serial.print(gx); Serial.print("\t");
   Serial.print(gy); Serial.print("\t");
   Serial.print(gz);Serial.print("\t");
   
   Serial.print("mag:\t");
   Serial.print(mx); Serial.print("\t");
   Serial.print(my); Serial.print("\t");
   Serial.print(mz); Serial.print("\t");
   
// To calculate heading in degrees. 0 degree indicates North
   float heading = atan2(-mx, my);
   if(heading < 0)
     heading += 2 * M_PI;
   Serial.print("heading:\t");
   Serial.println(heading * 180/M_PI);

   // blink LED to indicate activity
   blinkState = !blinkState;
   digitalWrite(LED_PIN, blinkState);
}