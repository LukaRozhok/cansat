#include <Wire.h> //Needed for I2C to GNSS

#include <SPI.h>

#include <LoRa.h>

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

#include "Adafruit_SHT4x.h"

SFE_UBLOX_GNSS myGNSS;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

#define myWire Wire // Connect using the Wire1 port. Change this if required

//#define myWire1 Wire1
  const int csPin = 17;
  const int resetPin = 1;
  const int irqPin = 0;
  byte msgCount = 0;
#define gnssAddress 0x42 // The default I2C address for u-blox modules is 0x42. Change this if required

void setup()
{
  Serial.begin(115200);
  delay(1000); 
  Serial.println("SparkFun u-blox Example");

  SPI.setRX(16);
  SPI.setCS(17);
  SPI.setSCK(18);
  SPI.setTX(19);
  SPI.begin();
  delay(1000);

  LoRa.setPins(csPin, resetPin, irqPin);  

  Serial.println("LoRa Sender Test");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Connected!");

  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  delay(1000);

    Serial.println("Adafruit SHT4x test");
  if (! sht4.begin(&Wire1)) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);

  myWire.setSDA(4);
  myWire.setSCL(5);
  myWire.begin(); // Start I2C
  delay(1000);
  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myGNSS.begin(myWire, gnssAddress) == false) //Connect to the u-blox module using our custom port and address
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay (1000);
  }
  Serial.println("GNSS detected");
  //myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  
  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR
}

void loop()
{
  int32_t latitude;
  int32_t longitude;
  int32_t altitude ;
  Serial.println("started iteration");
  // Request (poll) the position, velocity and time (PVT) information.
  // The module only responds when a new position is available. Default is once per second.
  // getPVT() returns true when new data is received.
  if (myGNSS.getPVT() == true)
  {
    latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    Serial.println();
  }
  else {
    Serial.println("no PVT response");
  }

  sensors_event_t humidity, temp;
  
  uint32_t timestamp = millis();
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  timestamp = millis() - timestamp;

  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  Serial.print("Read duration (ms): ");
  Serial.println(timestamp);

  Serial.println("Lora sent started");
  LoRa.beginPacket();
  LoRa.print("Temperature: ");
  LoRa.print(temp.temperature);
  LoRa.print(" degrees C, Humidity: ");
  LoRa.print(humidity.relative_humidity);
  LoRa.print("% rH");
  LoRa.print(" Latitude ");
  LoRa.print(latitude);
  LoRa.print(" Longitude ");
  LoRa.print(longitude);
  LoRa.print(" Altitude ");
  LoRa.print(altitude);
  LoRa.endPacket();
  Serial.println("Lora sent ended");
  msgCount++;

  delay(5000);
}