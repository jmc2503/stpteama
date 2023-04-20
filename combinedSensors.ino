//Libraries
#include <Adafruit_MPL3115A2.h>
#include "Adafruit_PM25AQI.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "DFRobot_MICS.h"
#include <Adafruit_INA219.h>
#include <SPI.h>
#include <SD.h>

File myFile;


//Constants
#define CALIBRATION_TIME 0.1
#define Mics_I2C_ADDRESS 0x75

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

int tempo = 180;

int melody[] = {

  // Nokia Ringtone 
  // Score available at https://musescore.com/user/29944637/scores/5266155
  
  NOTE_E5, 8, NOTE_D5, 8, NOTE_FS4, 4, NOTE_GS4, 4, 
  NOTE_CS5, 8, NOTE_B4, 8, NOTE_D4, 4, NOTE_E4, 4, 
  NOTE_B4, 8, NOTE_A4, 8, NOTE_CS4, 4, NOTE_E4, 4,
  NOTE_A4, 2, 
};


//Sensor Definitions
Adafruit_MPL3115A2 baro;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI(); //Big Blue Thing
Adafruit_GPS GPS(&Wire);
DFRobot_MICS_I2C mics(&Wire, Mics_I2C_ADDRESS); //NO2 Gas Sensor
Adafruit_INA219 ina219; //Current Sensor

SoftwareSerial pmSerial(0, 1); //For the PM2.5 sensor

const int chipselect = BUILTIN_SDCARD;

const int buzzer = 32;

uint32_t timer = millis();

int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Combined Sensor Test");

  pinMode(buzzer, OUTPUT);

  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, melody[thisNote], noteDuration * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(buzzer);
  }

  
  //NO2 sensor calibration
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

  //INA219
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }


  Serial.println("PM25 found!");

  //SD Card
  if (!SD.begin(chipselect)) {
    Serial.println("initialization failed!");
    while (1);
  }


  baro.setSeaPressure(1013.26);

  delay(1000);
}

void loop() {
  //Altimeter Variables
  float pressure = baro.getPressure();
  float altitude = baro.getAltitude();
  float temperature = baro.getTemperature();

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  PM25_AQI_Data data;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);


  myFile = SD.open("test.txt", FILE_WRITE);

  if(myFile){

    myFile.println(timer);

    //Gas: NO2, CO
    myFile.println("Gas: ");
    float gasdata = mics.getGasData(NO2);
    float gasdata2 = mics.getGasData(CO);
    myFile.print(gasdata,1);
    myFile.print(gasdata2,1);

    //Altimeter: pressure hPa, altitude m, temperature C
    myFile.println("Altimeter: ");
    myFile.print(pressure); myFile.print(" hPa, ");
    myFile.print(altitude); myFile.print(" m, ");
    myFile.print(temperature); myFile.print(" C");

    //PM2.5: standard, /0.1L air
    myFile.println("PM2.5: ");
    myFile.print(data.pm25_standard);
    //Particles > 2.5um / 0.1L air:
    myFile.println(data.particles_25um);

    //GPS: time
    myFile.println("GPS");
    myFile.print("Time: ");
    if (GPS.hour < 10) { myFile.print('0'); }
    myFile.print(GPS.hour, DEC); myFile.print(':');
    if (GPS.minute < 10) { myFile.print('0'); }
    myFile.print(GPS.minute, DEC); myFile.print(':');
    if (GPS.seconds < 10) { myFile.print('0'); }
    myFile.print(GPS.seconds, DEC); myFile.print('.');
    if (GPS.milliseconds < 10) {
        myFile.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      myFile.print("0");
    }

    //Current: Bus, Shunt, Load, Current, Power
    myFile.println("Current/Power: ");
    myFile.print(busvoltage); myFile.println(" V");
    myFile.print(shuntvoltage); myFile.println(" mV");
    myFile.print(loadvoltage); myFile.println(" V");
    myFile.print(current_mA); myFile.println(" mA");
    myFile.print(power_mW); myFile.println(" mW");

    tone(buzzer, 440);
    delay(1000);
    noTone(buzzer);
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  myFile.close();
}

