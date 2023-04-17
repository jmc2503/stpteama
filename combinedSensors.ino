//Libraries
#include <Adafruit_MPL3115A2.h>
#include "Adafruit_PM25AQI.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "DFRobot_MICS.h"

//Constants
#define CALIBRATION_TIME 0.1
#define Mics_I2C_ADDRESS 0x75

//Sensor Definitions
Adafruit_MPL3115A2 baro;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
Adafruit_GPS GPS(&Wire);
DFRobot_MICS_I2C mics(&Wire, Mics_I2C_ADDRESS);

SoftwareSerial pmSerial(0, 1); //For the PM2.5 sensor


uint32_t timer = millis();

void setup() {
  Serial.begin(9600);
  Serial.println("Combined Sensor Test");

  //NO2 sensor calibration
  while(!Serial);
  while(!mics.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  } Serial.println("Device connected successfully !");

  uint8_t mode = mics.getPowerState();
  if(mode == SLEEP_MODE){
    mics.wakeUpMode();
    Serial.println("wake up sensor success!");
  }else{
    Serial.println("The sensor is wake up mode");
  }

  while(!mics.warmUpTime(CALIBRATION_TIME)){
    Serial.println("Please wait until the warm-up time is over!");
    delay(1000);
  }

  GPS.begin(0x10);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);


  //Altimeter
  if (!baro.begin()) {
    Serial.println("Could not find altimeter");
    while(1);
  }

  //PM2.5
  if (! aqi.begin_UART(&Serial1)) { // connect to the sensor over hardware serial
    Serial.println("Could not find PM 2.5 sensor!");
    while (1) delay(10);
  }

  Serial.println("PM25 found!");

  baro.setSeaPressure(1013.26);

  delay(1000);
}

void loop() {
  //Altimeter Variables
  float pressure = baro.getPressure();
  float altitude = baro.getAltitude();
  float temperature = baro.getTemperature();

  float gasdata = mics.getGasData(NO2);
  Serial.println("---------Gas Sensor---------");
  Serial.print(gasdata,1);
  Serial.println(" PPM");

  //Altimeter Prints
  Serial.println("---------Altimeter---------");
  Serial.print("pressure = "); Serial.print(pressure); Serial.println(" hPa");
  Serial.print("altitude = "); Serial.print(altitude); Serial.println(" m");
  Serial.print("temperature = "); Serial.print(temperature); Serial.println(" C");

  PM25_AQI_Data data;
  
  if (!aqi.read(&data)) {
    Serial.println("Could not read from AQI");
  }
  Serial.println("AQI reading success");

  Serial.println();
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Concentration Units (standard)"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_standard);
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_standard);
  Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_standard);
  Serial.println(F("Concentration Units (environmental)"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_env);
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_env);
  Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_env);
  Serial.println(F("---------------------------------------"));
  Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(data.particles_03um);
  Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(data.particles_05um);
  Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(data.particles_10um);
  Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(data.particles_25um);
  Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(data.particles_50um);
  Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(data.particles_100um);
  Serial.println(F("---------------------------------------"));

  Serial.println("---------GPS---------");
  Serial.print("\nTime: ");
  if (GPS.hour < 10) { Serial.print('0'); }
  Serial.print(GPS.hour, DEC); Serial.print(':');
  if (GPS.minute < 10) { Serial.print('0'); }
  Serial.print(GPS.minute, DEC); Serial.print(':');
  if (GPS.seconds < 10) { Serial.print('0'); }
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  if (GPS.milliseconds < 10) {
      Serial.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    Serial.print("0");
  }

  delay(1000);
}

