#include <SPI.h>
#include <LoRa.h>
#include "Adafruit_SHT4x.h"

const int csPin = 4;
const int resetPin = 2;
const int irqPin = 3;
byte msgCount = 0;

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

void setup() {
  Serial.begin(9600);
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);

  Serial.println("LoRa Sender Test");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1);
  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(msgCount);

  sensors_event_t humidity, temp;
  sht4.getEvent(&humidity, &temp);

  LoRa.beginPacket();
  LoRa.print("Temperature: ");
  LoRa.print(temp.temperature);
  LoRa.print(" degrees C, Humidity: ");
  LoRa.print(humidity.relative_humidity);
  LoRa.print("% rH");
  LoRa.endPacket();

  msgCount++;
  delay(5000);
}