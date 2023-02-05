#include <CanSatKit.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

using namespace CanSatKit;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x20 for a 16 chars and 2 line display
//int counter = 0;

// set radio receiver parameters - see comments below
// remember to set the same radio parameters in
// transmitter and receiver boards!
Radio radio(Pins::Radio::ChipSelect,
            Pins::Radio::DIO0,
            433.0,                  // frequency in MHz
            Bandwidth_125000_Hz,    // bandwidth - check with CanSat regulations to set allowed value
            SpreadingFactor_9,      // see provided presentations to determine which setting is the best
            CodingRate_4_8);        // see provided presentations to determine which setting is the best

void setup() {
  SerialUSB.begin(115200);

  // start radio module  
  radio.begin();

  lcd.init();
  lcd.backlight();  
}

void loop() {
  // prepare empty space for received frame
  // maximum length is maximum frame length + null termination
  // 255 + 1 byte = 256 bytes
  char data[256];
  //char metrics[20];

  // receive data and save it to string
  radio.receive(data);
  
  // get and print signal level (rssi)
  SerialUSB.print("Recieved (RSSI = ");
  SerialUSB.print(radio.get_rssi_last());
  SerialUSB.print("): ");

  // print received message
  SerialUSB.println(data);

  lcd.setCursor(0, 0);
  lcd.print(data);
  //lcd.setCursor(0, 1);
  //lcd.print(counter);
  //counter += 1;  
}
