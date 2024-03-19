#include <Wire.h> //Needed for I2C to GNSS

#include <Servo.h>
Servo myservo;  // create servo object to control a servo

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

#include "pico/stdlib.h"

SFE_UBLOX_GNSS myGNSS;

#define gnssAddress 0x42 // The default I2C address for u-blox modules is 0x42. Change this if required

#define myWire Wire // Connect using the Wire1 port. Change this if required

const uint RED_PIN = 16;
const uint GREEN_PIN = PICO_DEFAULT_LED_PIN;
const uint YELLOW_PIN = 17;

const uint PARACHUTE_LOW_BOUNDARY = 1000;
const uint PARACHUTE_HIGH_BOUNDARY = 1500;

int gpsReadings = 0; // Licznik odczytów GPS
bool gpsReady = false; // Flaga oznaczająca gotowość GPS
int goingDownCounter = 0;
bool parachuteOpen = false;

int32_t ALT_PRECISION = 10; // meter(s)
int32_t altitude_current, altitude_prev, altitude_zero;

void init_pin(uint pin){
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
}

void setup()
{
  init_pin(GREEN_PIN);
  init_pin(YELLOW_PIN);
  init_pin(RED_PIN);
  myservo.attach(10);  // attaches the servo on GIO10 to the servo object

  Serial.begin(115200);
  delay(1000); 
  Serial.println("Servo Gnss test has started!");

  myWire.setSDA(4);
  myWire.setSCL(5);
  myWire.begin();   
  delay(1000);

while (myGNSS.begin(myWire, gnssAddress) == false) 
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay (1000);
  }
  Serial.println("GNSS detected");
  altitude_zero = 100; // above the ground level, in meters
  altitude_prev = 0;

}

void openParachute() {
  Serial.println("servo position 0");
  myservo.write(0);
  delay(535); //360 rotation for 5V - delay 535
  Serial.println("servo position 90");
  myservo.write(90);
  parachuteOpen = true;
}

void light_on(uint pin){
  gpio_put(pin, 1);
  sleep_ms(250);
  gpio_put(pin, 0);
  sleep_ms(250);
}

void loop()
{
  Serial.println("started iteration");

  if (myGNSS.getPVT() == true)
  {
    altitude_current = myGNSS.getAltitudeMSL() / 1000; // Altitude above Mean Sea Level in meters
    Serial.print(F(" Alt current: "));
    Serial.print(altitude_current);
    Serial.print(F(" (m)"));
    Serial.println();

    // Sprawdź, czy odbiornik GPS otrzymał co najmniej 3 odczyty
    if (!gpsReady && gpsReadings >= 3) {
      gpsReady = true;
    }

    // Jeśli odbiornik GPS jest gotowy i wysokość jest dodatnia, wykonaj dalszą analizę
    if (gpsReady) {
      if (altitude_prev - altitude_current > ALT_PRECISION) {
        Serial.println("going down");
        goingDownCounter++;
        light_on(RED_PIN);

        if (!parachuteOpen && goingDownCounter >= 3 && 
          altitude_current - altitude_zero < PARACHUTE_HIGH_BOUNDARY &&
          altitude_current - altitude_zero > PARACHUTE_LOW_BOUNDARY
          ) {
          Serial.println("we are at 1km above the ground level, open the parachute");
          openParachute();
        }
      } else if (altitude_current - altitude_prev > ALT_PRECISION) {
        Serial.println("going up");
        goingDownCounter = 0;
        light_on(GREEN_PIN);
      } else {
        Serial.println("not moving");
        goingDownCounter = 0;
        light_on(YELLOW_PIN);
      }
    } else {
      // Jeśli odbiornik GPS nie jest jeszcze gotowy lub wysokość jest nieprawidłowa, zwiększ licznik odczytów GPS
      Serial.println("GPS not ready");
    }
    if (altitude_current > 0 && !gpsReady) {
       Serial.println("positive altitude detected, getting ready...");
       gpsReadings++;
    }
    else {
       gpsReadings = 0;
    }
  } else {
    Serial.println("no PVT response");
  }

  delay(5000);
  altitude_prev = altitude_current;
}
