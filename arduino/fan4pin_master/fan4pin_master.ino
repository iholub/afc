// Simple I2C protocol for Arduino
// Master side program
// (c) 2014 Ignas Gramba
//

#include <Wire.h>
void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop()
{
  int x = GetXSensorValue(0x31); // 1 - slave's address
  Serial.print("X Sensor value: ");
  Serial.println(x);
  
  delay (100);
}

int GetXSensorValue(byte SlaveDeviceId){

  // SEND COMMAND 
  Wire.beginTransmission(SlaveDeviceId);
  //Wire.write(1); // Transfer command ("1") to get X sensor value;
  //delay(10);

  // GET RESPONSE
  int receivedValue;
  int available = Wire.requestFrom(SlaveDeviceId, (byte)2);
  if(available == 2)
  {
    receivedValue = Wire.read() << 8 | Wire.read(); // combine two bytes into integer
  }
  else
  {
    Serial.print("ERROR: Unexpected number of bytes received (XSensorValue): ");
    Serial.println(available);
  }
  Wire.endTransmission();
  
  return receivedValue;
} 
