/**
 * Async example for MPL3115A2
 */
/*
#include <Adafruit_MPL3115A2.h>

Adafruit_MPL3115A2 mpl;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Adafruit_MPL3115A2 test!");

  if (!mpl.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  // set mode before starting a conversion
  Serial.println("Setting mode to barometer (pressure).");
  mpl.setMode(MPL3115A2_BAROMETER);
}

void loop() {
  // start a conversion
  Serial.println("Starting a conversion.");
  mpl.startOneShot();

  // do something else while waiting
  Serial.println("Counting number while waiting...");
  int count = 0;
  while (!mpl.conversionComplete()) {
    count++;
  }
  Serial.print("Done! Counted to "); Serial.println(count);

  // now get results
  Serial.print("Pressure = ");
  Serial.println(mpl.getLastConversionResults(MPL3115A2_PRESSURE));
  Serial.print("Temperature = ");
  Serial.println(mpl.getLastConversionResults(MPL3115A2_TEMPERATURE));
  Serial.println();

  delay(1000);
}
*/
/**************************************************************************/
/*!
    @file     Adafruit_MPL3115A2.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)
    Example for the MPL3115A2 barometric pressure sensor
    This is a library for the Adafruit MPL3115A2 breakout
    ----> https://www.adafruit.com/products/1893
    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
    @section  HISTORY
    v1.0 - First release
*/
/**************************************************************************/

#include <Adafruit_MPL3115A2.h>

Adafruit_MPL3115A2 baro;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Adafruit_MPL3115A2 test!");

  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  // use to set sea level pressure for current location
  // this is needed for accurate altitude measurement
  // STD SLP = 1013.26 hPa
  baro.setSeaPressure(1013.26);
}

void loop() {
  float pressure = baro.getPressure();
  float altitude = baro.getAltitude();
  float temperature = baro.getTemperature();

  Serial.println("-----------------");
  Serial.print("pressure = "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("altitude = "); Serial.print(altitude); Serial.println(" m");
  Serial.print("temperature = "); Serial.print(temperature); Serial.println(" C");

  delay(250);
}