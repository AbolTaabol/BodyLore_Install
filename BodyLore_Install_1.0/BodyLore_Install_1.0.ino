/***************************************************

 ****************************************************/

#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

int analogPin[] = {A0, A1, A2};

int sensorThresholdHigh = 200;
int sensorThresholdLow = 30;

int sensorValue[] = {0, 0, 0};

int sensorSwitch[] = {0, 0, 0};

int playFlag = 0;


// variables:
int sensorOff[] = {0, 0, 0};         // maximum sensor value

unsigned long previousMillis = 0;        // will store last time LED was updated

long interval = 10;

#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266))   // Using a soft serial port
#include <SoftwareSerial.h>
SoftwareSerial softSerial(/*rx =*/10, /*tx =*/11);
#define FPSerial softSerial
#else
#define FPSerial Serial1
#endif

DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

void setup() {

#if (defined ESP32)
  FPSerial.begin(9600, SERIAL_8N1, /*rx =*/D3, /*tx =*/D2);
#else
  FPSerial.begin(9600);
#endif

  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true) {
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }

  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(30);  //Set volume value. From 0 to 30

  Serial.println(F("Calibrating Jars..."));

  calibrateSensor();

  Serial.print("MP3 play status: ");
  Serial.println(myDFPlayer.readState());

}

/*This is the main loop*/
void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;

    sensorValue[0] = analogRead(analogPin[0]);
    sensorValue[1] = analogRead(analogPin[1]);
    sensorValue[2] = analogRead(analogPin[2]);

    //printSensorDifference();
    //printSensorValue();

    scanJarState();

    if (myDFPlayer.available()) {
      printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
    }
  }

}

void scanJarState() {

  for (int i = 0; i < 3; i++) {
    if (abs(sensorValue[i] - sensorOff[i]) > sensorThresholdHigh && sensorSwitch[i] == 0) {

      sensorSwitch[i] = 1;
      Serial.print("Jar ");
      Serial.print(i + 1);
      Serial.println(" On");


    }

    if (abs(sensorValue[i] - sensorOff[i]) < sensorThresholdLow && sensorSwitch[i] == 1) {

      sensorSwitch[i] = 0;
      Serial.print("Jar ");
      Serial.print(i + 1);
      Serial.println(" Off");

    }

  }

  controlNarration();

}

void controlNarration() {

  int counter = 0;
  int playNumber = 0;

  for (int i = 0; i < 3; i++) {

    if (sensorSwitch[i] == 1) {
      counter++;

      if (counter == 1) {
        playNumber = i + 1;
      }
    }

  }

  if (counter == 1 && playFlag == 0) {
    playFlag = 1;
    myDFPlayer.play(playNumber);
    Serial.print("Starting Jar ");
    Serial.println(playNumber);
  }

  if (counter != 1 && playFlag == 1) {
    playFlag = 0;
    myDFPlayer.pause();
  }

}

void printSensorDifference() {
  Serial.print("Difference 1: ");
  Serial.print(abs(sensorValue[0] - sensorOff[0]));
  Serial.print(",");
  Serial.print("Difference 2: ");
  Serial.print(abs(sensorValue[1] - sensorOff[1]));
  Serial.print(",");
  Serial.print("Difference 3: ");
  Serial.println(abs(sensorValue[2] - sensorOff[2]));
}

void printSensorValue() {

  Serial.print("Sensor Values: ");
  Serial.print(sensorValue[0]);
  Serial.print(",");
  Serial.print(sensorValue[1]);
  Serial.print(",");
  Serial.println(sensorValue[2]);

}

void calibrateSensor() {

  while (millis() < 5000) {

    sensorValue[0] = analogRead(analogPin[0]);
    sensorValue[1] = analogRead(analogPin[1]);
    sensorValue[2] = analogRead(analogPin[2]);

    // record the maximum sensor value
    if (sensorValue[0] > sensorOff[0]) {

      sensorOff[0] = sensorValue[0];

    }

    if (sensorValue[1] > sensorOff[1]) {

      sensorOff[1] = sensorValue[1];

    }

    if (sensorValue[2] > sensorOff[2]) {

      sensorOff[2] = sensorValue[2];

    }

  }

  Serial.print("Jar 1 off value: ");
  Serial.println(sensorOff[0]);
  Serial.print("Jar 2 off value: ");
  Serial.println(sensorOff[1]);
  Serial.print("Jar 3 off value: ");
  Serial.println(sensorOff[2]);

}

void printDetail(uint8_t type, int value) {

  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      playFlag = 0;
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }

}
