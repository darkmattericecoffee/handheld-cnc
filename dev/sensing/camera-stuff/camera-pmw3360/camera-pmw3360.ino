#include <PMW3360.h>
/* 
Frame capture (=Camera) mode:
This mode disables navigation and overwrites any downloaded firmware.
A hardware reset is required to restore navigation, and the firmware must be reloaded.

# PIN CONNECTION
  * MI = MISO
  * MO = MOSI
  * SS = Slave Select / Chip Select
  * SC = SPI Clock
  * MT = Motion (active low interrupt line)
  * RS = Reset
  * GD = Ground
  * VI = Voltage in up to +5.5V 

Module   Arduino
  RS --- (NONE)
  GD --- GND
  MT --- (NONE)
  SS --- Pin_10   (use this pin to initialize a PMW3360 instance)
  SC --- SCK 
  MO --- MOSI
  MI --- MISO
  VI --- 5V
*/

#define SS  30   // Slave Select pin. Connect this to SS on the module.

PMW3360 sensor;

void setup() {
  Serial.begin(9600);  
  while(!Serial);
  
  if(sensor.begin(SS))  // 10 is the pin connected to SS of the module.
    Serial.println("Sensor initialization successed");
  else
    Serial.println("Sensor initialization failed");

  // wait for 250 ms for frame capture.
  delay(250);
}

void loop() {
  // The following routine shoud be alwyas performed together.
  // BEGIN -------------------------------------------------
  long timeIn = micros();
  sensor.prepareImage();
  for(int i=0;i<36*36;i++)
  {
    byte pixel = sensor.readImagePixel();
    Serial.print(pixel, DEC);
    Serial.print(' ');
  }
  sensor.endImage();
  // END ----------------------------------------------------
  // Serial.printf("Image time = %ius\n", micros() - timeIn);
  delay(50);

  // optional: Surface quality report
  int squal = sensor.readReg(REG_SQUAL);
  Serial.println(squal);

  delay(10);
}