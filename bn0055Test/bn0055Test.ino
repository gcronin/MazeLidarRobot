#include <I2C.h>

// BN0055 specific registers
#define    BN0055_ADDRESS   0x28          // Default I2C Address of BN0055.
#define    CHIP_ID          0x00          // should contain the value 0xA0
#define    OPR_MODE         0x3D          // config, accelerometer only, fusion modes, etc.
#define    SYS_TRIGGER      0x3F          // used to reset, for external crystal oscillator
#define    POWER_MODE       0x3E          //
#define    PAGE_ID          0x07          // there are 2 pages of registers, this tells which page you are on
#define    EULER_X_LSB      0X1A          // Euler Angle registers starting point
#define    TEMP_ADDR        0x34          // Temperature Register

byte value = -100;
double head, pitch, roll;  //Euler Angles

void setup() {
  Serial.begin(9600);
  I2c.begin(); // Opens & joins the irc bus as master
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  
  if(BN0055setup()) { Serial.println("Setup BN0055 Successful"); }

  value = -100;
  i2cRead(BN0055_ADDRESS, TEMP_ADDR, 1, &value);
  Serial.print("Temp: ");
  Serial.println(value);

  if(BN0055useCrystal()) { Serial.println("BN0055 Using External Clock"); }
  BN0055selfTest();
}

void loop() {
  BN0055ReadEuler();
  BN0055getCalibration();
  Serial.print("X: ");
  Serial.print(head);
  Serial.print(" Y: ");
  Serial.print(pitch);
  Serial.print(" Z: ");
  Serial.print(roll);
  delay(100);
}

int BN0055setup()
{
  i2cRead(BN0055_ADDRESS, CHIP_ID, 1, &value); // Read Chip ID, should be 0xA0
  if(value == 160) { Serial.println("Found BN0055"); }
  else { return 0; }

  value = -100;
  i2cWrite(BN0055_ADDRESS, OPR_MODE, 0);  // Set operation mode to configure
  i2cRead(BN0055_ADDRESS, OPR_MODE, 1, &value);
  Serial.print("OPR_MODE: ");
  Serial.println(value);

  i2cWrite(BN0055_ADDRESS, SYS_TRIGGER, 0x20);   // reset the sensor
  while( value!=160) { i2cRead(BN0055_ADDRESS, CHIP_ID, 1, &value); }
  Serial.println("Reset BN0055");


  value = -100;
  i2cWrite(BN0055_ADDRESS, POWER_MODE, 0); // Set power mode to normal
  i2cRead(BN0055_ADDRESS, POWER_MODE, 1, &value);
  Serial.print("Power Mode: ");
  Serial.println(value);
  
  value = -100;
  i2cWrite(BN0055_ADDRESS, PAGE_ID, 0); // Go to Page 0 of memory registers
  i2cRead(BN0055_ADDRESS, PAGE_ID, 1, &value);
  Serial.print("Page ID: ");
  Serial.println(value);
  
  i2cWrite(BN0055_ADDRESS, SYS_TRIGGER, 0); // no idea what this does
  value = -100;
  i2cWrite(BN0055_ADDRESS, OPR_MODE, 0X0C);  // set mode to Fusion NDOF
  i2cRead(BN0055_ADDRESS, OPR_MODE, 1, &value);
  Serial.print("OPR_MODE: ");
  Serial.println(value);
  return 1;
}

boolean BN0055useCrystal()
{
  i2cWrite(BN0055_ADDRESS, OPR_MODE, 0);  // Set operation mode to configure
  delay(30);
  delay(25);
  i2cWrite(BN0055_ADDRESS, PAGE_ID, 0); // Go to Page 0 of memory registers
  i2cWrite(BN0055_ADDRESS, SYS_TRIGGER, 0x80);  //An External clock can be selected by setting bit CLK_SEL in the SYSTEM_TRIGGER register.) 0x80 = 10000000
  delay(50);
  i2cWrite(BN0055_ADDRESS, SYS_TRIGGER, 0x80);  //An External clock can be selected by setting bit CLK_SEL in the SYSTEM_TRIGGER register.) 0x80 = 10000000
  delay(50);
  i2cWrite(BN0055_ADDRESS, SYS_TRIGGER, 0x80);  //An External clock can be selected by setting bit CLK_SEL in the SYSTEM_TRIGGER register.) 0x80 = 10000000
  delay(50);
  i2cWrite(BN0055_ADDRESS, SYS_TRIGGER, 0x01); // triggers a self test
  delay(50);
  BN0055selfTest();
  i2cWrite(BN0055_ADDRESS, OPR_MODE, 0X0C);  // set mode the Fusion NDOF
  delay(30);
  delay(20);
  BN0055selfTest();
  return 1;
}


void BN0055selfTest()
{
  value = -100;
  i2cRead(BN0055_ADDRESS, 0X39, 1, &value);
  Serial.print("System Status: ");
  Serial.print(value);
  value = -100;
  i2cRead(BN0055_ADDRESS, 0X36, 1, &value);
  Serial.print("  Self Test Result: ");
  Serial.print(value);
  value = -100;
  i2cRead(BN0055_ADDRESS, 0X3A, 1, &value);
  Serial.print("  System Error: ");
  Serial.println(value);
}

void BN0055ReadEuler()
{
  uint8_t buffer[6];
  memset (buffer, 0, 6);  // set all buffer values to 0
  int16_t x, y, z;

  i2cRead(BN0055_ADDRESS, EULER_X_LSB, 6, &buffer[0]);
  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

   /* 1 degree = 16 LSB , (section 3.6.4)*/
  head  = ((double)x)/16.0;
  pitch = ((double)y)/16.0;
  roll = ((double)z)/16.0;
}

void BN0055getCalibration()
{
  value = -100;
  byte reading;
  i2cRead(BN0055_ADDRESS, 0x35, 1, &value);
  reading = (value >> 6) & 0x03;
  Serial.print("    System: ");
  Serial.print(reading);
  reading = (value >> 4) & 0x03;
  Serial.print(" Gyro: ");
  Serial.print(reading);
  reading = (value >> 2) & 0x03;
  Serial.print(" Accel: ");
  Serial.print(reading);
  reading = value & 0x03;
  Serial.print(" Mag: ");
  Serial.println(reading);
}


uint8_t i2cWrite(byte address, byte memoryRegister, byte value)
{
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK responses
  while (nackack != 0){
    nackack = I2c.write(address, memoryRegister, value);
     delay(1); // Wait 1 ms to prevent overpolling 
  }
  delay(20);
  return nackack;
}

uint8_t i2cRead(byte address, byte memoryRegister, byte numBytes, char *dataBuffer )
{
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK responses
  while (nackack != 0){
    nackack = I2c.read(address, memoryRegister, numBytes, (uint8_t*)dataBuffer);
     delay(1); // Wait 1 ms to prevent overpolling 
  }
  delay(20);
  return nackack;
}
