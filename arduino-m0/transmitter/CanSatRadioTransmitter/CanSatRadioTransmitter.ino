/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <CanSatKit.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

using namespace CanSatKit;

int counter = 1;

Adafruit_BMP3XX bmp;

Radio radio(Pins::Radio::ChipSelect,
            Pins::Radio::DIO0,
            433.0,
            Bandwidth_125000_Hz,
            SpreadingFactor_9,
            CodingRate_4_8);
Frame frame;            

void setup() {
  SerialUSB.begin(115200);
  //while (!Serial);
  SerialUSB.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    SerialUSB.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  SerialUSB.println("Connected to BMP3 sensor!");
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  radio.begin();
}

void loop() {
  if (! bmp.performReading()) {
    SerialUSB.println("Failed to perform reading :(");
    return;
  }

  frame.print(bmp.temperature);
  frame.print("C, ");
  frame.print(bmp.pressure / 100.0);
  frame.print("hPa - sent from transmitter, message #");
  frame.print(counter);
  // increment counter variable
  counter++;
  //SerialUSB.print(bmp.temperature); //deg C
  //SerialUSB.print(bmp.pressure / 100.0); //hPa
  //SerialUSB.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)); //m
  radio.transmit(frame);

  SerialUSB.print(frame);  
  SerialUSB.println(" -> transmitted.");
  frame.clear();
  delay(4000);
}
