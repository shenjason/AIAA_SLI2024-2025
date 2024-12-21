#include <Wire.h>
#include "Adafruit_SGP30.h"
#include <SD.h>
#include <SPI.h>

#define BUZZER 23

Adafruit_SGP30 sgp;

char segment[300];
char segment_f[300];
bool newsegment;
double gpsLat; double gpsLong;
char gpsLatDir; char gpsLongDir;
bool fix; bool isMoxData;
int seg_i;
int nofixticks; int fixticks; int noparseticks; int moxTicks;
float TVOC; float eCO2;
long prevTime;
elapsedMillis timeElapsed;

File dataFile;


void setup() {
  pinMode(BUZZER, OUTPUT);
  Serial.begin(115200);
  Serial3.begin(115200);

  beep(2);
  
  // Serial3.println("freset");
  Serial3.println("unmask GLO");
  Serial3.println("unmask GAL");
  Serial3.println("unmask BDS");
  Serial3.println("unmask GPS");
  Serial3.println("CONFIG COM1 115200 8 n 1");
  Serial3.println("GPGGA 0.05");
  Serial3.println("saveconfig");

  if (!sgp.begin()){
    delay(1000);
    beep(0.1);
    while (true) {};
  }

  if (!SD.begin(BUILTIN_SDCARD)){
    delay(1000);
    beep(0.1);
    delay(300);
    beep(0.1);
    while (true) {};
  }

  dataFile = SD.open("Data.csv", FILE_WRITE);

  dataFile.print("Time Elapsed(miliseconds)");
  dataFile.print(", ");
  dataFile.print("Latitude(ddeg)");
  dataFile.print(", ");
  dataFile.print("Longitude(ddeg)");
  dataFile.print(", ");
  dataFile.print("TVOC(ppb\t)");
  dataFile.print(", ");
  dataFile.println("eCO2(ppm)");
  
  dataFile.close();

  delay(1000);
  beep(0.1);
  delay(300);
  beep(0.1);
  delay(300);
  beep(0.1);

  newsegment = false;
  nofixticks = 0;
  fixticks = 0;
  noparseticks = 0;
  seg_i = 0;
  isMoxData = false;
  moxTicks = 0;
  timeElapsed = 0;
  prevTime = 0;
}

void loop() {

  UpdateGPS();
  // UpdateMox();

  if (nofixticks > 80){
    beep(0.1);
    nofixticks = 0;
  }

  if (fixticks > 200){
    beep(0.1);
    delay(100);
    beep(0.1);
    fixticks = 0;
  }

  if (noparseticks > 80){
    beep(0.5);
    noparseticks = 0;
  }

  if ((fix || isMoxData) && (timeElapsed - prevTime) > 1000){
    prevTime = timeElapsed;
    dataFile = SD.open("Data.csv", FILE_WRITE);
    dataFile.print(timeElapsed);
    dataFile.print(", ");
    if (fix){
      dataFile.print(gpsLat, 7);
      dataFile.print(", ");
      dataFile.print(gpsLong, 7);
      dataFile.print(", ");
    }else{
      dataFile.print("N/A, N/A, ");
    }

    if (isMoxData){
        dataFile.print(TVOC);
        dataFile.print(", ");
        dataFile.print(eCO2);
    }else{
      dataFile.print("N/A, N/A");
    }

    dataFile.println();

    dataFile.close();
  }
  
}

void UpdateMox(){
  isMoxData = false;
  moxTicks++;
  if  (!sgp.IAQmeasure() || moxTicks < 80) return;
  moxTicks = 0;
  isMoxData = true;

  TVOC = sgp.TVOC;
  eCO2 = sgp.eCO2;
}



void UpdateGPS(){
  if (Serial3.available() == 0) return;

  char read = Serial3.read();
  // Serial.print(read);
  bool done = parseSegment(read);
  if (done){
    noparseticks = 0;
    // Serial.println(segment_f); 
    if (parseGPSNMEA()){
      if (!fix){
        nofixticks ++;
        return;
      }
      fixticks ++;
    }else{
      noparseticks ++;
    }
    return;
  }
  return;
}

bool parseSegment(char newchar){
  if (newchar == '$') {
    seg_i = 0;
    newsegment = true;
    memset(segment, 0, sizeof(segment));
    return false;
  }
  if (!newsegment) return false;
  if (newchar == '\r'){
    newsegment = false;
    if (seg_i > 100) return false;
    segment[seg_i] = '\n';
    strcpy(segment_f, segment);
    return true;
  }
  segment[seg_i] = newchar;
  seg_i++;

  return false;
}

bool parseGPSNMEA(){
  char* part = strtok(segment_f, ",");
  if (strcmp(part, "GNGGA") != 0) return false;
  
  int index = 1;
  while (part != NULL){
    part = strtok(NULL, ",");
    if (part == NULL) break;
    // Serial.println(part);
    if (index == 6){
      fix = true;
      if (part[0] == '0') fix = false;
    }
    if (index == 2){
      if (strlen(part) < 8) continue;
      String latMin = ""; String latDeg = "";
      latDeg += part[0]; latDeg += part[1];
      latMin += part[3]; latMin += part[4]; latMin += part[5]; latMin += part[6]; latMin += part[7];

      gpsLat = latDeg.toInt() + (latMin.toFloat() / 60.0);
    }
    if (index == 4){
      if (strlen(part) < 9) continue;
      String longMin = ""; String longDeg = "";
      longDeg += part[0]; longDeg += part[1]; longDeg += part[2];
      longMin += part[4]; longMin += part[5]; longMin += part[6]; longMin += part[7]; longMin += part[8];

      gpsLong = longDeg.toInt() + (longMin.toFloat() / 60.0);
    }
    if (index == 3){
      if (strlen(part) < 1) continue;
      gpsLatDir = part[0];
    } 
    if (index == 5){
      if (strlen(part) < 1) continue;
      gpsLongDir = part[0];
    }
    index++;
  }
  // Serial.println(gpsLat, 8);
  // Serial.println(gpsLong, 8);
  return true;
}


void beep(float seconds){
  digitalWrite(BUZZER, HIGH);
  delay(seconds*1000);
  digitalWrite(BUZZER, LOW);
}
