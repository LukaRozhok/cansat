#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int csPin = 4;
const int resetPin = 2;
const int irqPin = 3;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adres wyświetlacza LCD i jego rozmiar

void setup() {
  Serial.begin(9600);
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);

  Serial.println("LoRa Receiver Test");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  lcd.init();         // Inicjalizacja wyświetlacza LCD
  lcd.backlight();    // Włączenie podświetlenia wyświetlacza LCD
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {
      char receivedChar = (char)LoRa.read();
      Serial.print(receivedChar);  // Wyświetlenie danych w terminalu szeregowym
      lcd.print(receivedChar);     // Wyświetlenie danych na wyświetlaczu LCD
    }
    Serial.println();
    lcd.setCursor(0, 1);  // Ustawienie kursora na początku drugiego wiersza
  }
}