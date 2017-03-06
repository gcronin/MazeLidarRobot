#include <Adafruit_TB6612.h>
#include <Servo.h>
#include <I2C.h>

// TB6612 pins
#define AIN1 9
#define BIN1 8
#define AIN2 10
#define BIN2 7
#define PWMA 11
#define PWMB 6



// LIDAR specific registers
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.


Servo LidarTurret;  // create servo object to control a servo
int distance = -1;  // lidar distance reading

void setup() {
  Serial.begin(9600);
  LidarTurret.attach(2);  // attaches the servo on pin 2 to the servo object
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
}

void loop() {
  LidarTurret.write(0);
  delay(500);
  readLidar();
  Serial.print("Right: ");
  Serial.print(distance);
  delay(1000);

  LidarTurret.write(85);
  delay(500);
  readLidar();
  Serial.print("  Ahead: ");
  Serial.print(distance);
  delay(1000);
  
  LidarTurret.write(174);
  delay(500);
  readLidar();
  Serial.print("  Left: ");
  Serial.println(distance);
  delay(1000);
}


void readLidar()
{  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
}

