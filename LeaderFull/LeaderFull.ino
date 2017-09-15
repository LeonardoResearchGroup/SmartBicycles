
//------------------------LIBRARIES------------------------\\

#include <SoftwareSerial.h> //viertual serial for the GPS
#include <TinyGPS++.h> //Helps with the handling of the GPS
#include <TinyBrite.h>
#include <Fat16.h>
//------------------------VARIABLES------------------------\\

//Debbugging
bool gpsOK = false;

//Speedometer
unsigned long time01;
double   bikeSpeed;
int sensorPin = 3;
double circuit;
double minSpeed;
//double wheels[4];

//Xbee Com
String beaconMsg;
long time4;
unsigned long timerBeaconLeader;
String linea;

//GPS
SoftwareSerial serialGPS(9, 10); // Creates virtual serial for the bluetooth - android GPS (RX, TX) pins,
TinyGPSPlus gps; //GPS object
String myLat = "0"; //My latitud
String myLng; //My longitud
long time2;

// Platoon Size its 4

int bikeID = 0; // bikes 1 throw 3 are fallowers, and 0 the leader
int beaconInterval = 1000;
int platoonInterval = 1000;
double desiredSpacing = 10;
double wheels[4];
double alphaSpeed[4];
//setup


//megabrite
#define clockpin  A3
#define latchpin  A5
#define datapin   A4
TinyBrite briteLED(1, TINYBRITE_AUTOUPDATE_ENABLE);

//SD Shield - / file setup
SdCard sd;
Fat16 myFile;
double expValues[40];

//controler
bool itsAlive = false;
int key = -1;
int oldkey = -1;
int runningExp = 1;


//------------------------SETUP------------------------\\

void setup() {
  //debugging

  Serial1.begin(9600); //USB serial

  //speedometer Setup

  pinMode(sensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), speedSensor, RISING);//interrupt for pin 2
  time01 = millis();


  //Xbee Com
  Serial1.begin(9600); //XBee serial1
  while (!Serial1) {
    ; // wait for serial port to connect.
  }



  //GPS
  serialGPS.begin(9600); //GPS SoftSerial


  time2 = millis();
  time4 = millis();

  // create the megabrite chain, as a global
  briteLED.setup(datapin, clockpin, latchpin);

  //SD card and experiments file read
  sd.begin(8);
  Fat16::init(&sd);
  if (!myFile.open("config.txt", O_READ)) {
    Serial.println("no abrio la SD o el archivo");
  }
  // read from the file until there's nothing else in it:
  char data;
  char line[6];
  int n = 0;
  while (myFile.fgets(line, sizeof(linea)) > 0) {
    String TEMP(line);
    expValues[n] = TEMP.toFloat();
    n++;
  }

  // close the file:
  myFile.close();
  //button control
  pinMode(0, INPUT);


}
//------------------------Program Loop------------------------\\

void loop() {
  readFromGPS();
  if (gpsOK) {
    control();
  }
  if (itsAlive) {
    sendBeacon();
    readBeacons();
    speedometer();
  } else {
    briteLED.sendColor(0, 0, 0);
  }
}

//------------------------Functions------------------------\\

//debugging


//speedometer
void speedSensor() {// this fuction its executeded each time the sensor interrupts arduino

  if (millis() - time01 > 100) {
    bikeSpeed = (circuit * 1000) / ((millis() - time01));
    if (millis() - time01 > 3000) {
      bikeSpeed = 0;
    }
    time01 = millis();
  }
}
void speedometer() {
  if (millis() - time01 > 2000) { //checks if the wheel hasn't move for 2 seconds and assumes it has stopt, resert speed to 0
    bikeSpeed = 0;
    briteLED.sendColor(0, 0, 512);


  }
  if (bikeSpeed > minSpeed + alphaSpeed[0]) {
    briteLED.sendColor(512, 0, 512);

  }
  if (bikeSpeed > 0 && bikeSpeed < minSpeed - alphaSpeed[0]) {
    briteLED.sendColor(0, 512, 512);
  }

  if (bikeSpeed > minSpeed - alphaSpeed[0] && bikeSpeed < minSpeed + alphaSpeed[0]) {
    briteLED.sendColor(0, 0, 512);
  }
}


//Xbee Com
void sendBeacon()// pruebas de envio de datos por serial un char a la vez
{
  if (millis() - time4 > beaconInterval-1) {
    time4 = millis();
    Serial1.print("0|" + myLat + "|" + myLng + "|" + bikeSpeed + "|" + String(gps.time.hour() - 5) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + '#'); //leader also sends its timer millis() after the
    myFile.println("0|" + myLat + "|" + myLng + "|" + bikeSpeed + "|" + String(gps.time.hour() - 5) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second())); //leader also sends its timer millis() after the

  }
}
void readBeacons() {
  while (Serial1.available()) {
    linea = Serial1.readStringUntil('#');
    myFile.println(linea);
  }
}


//GPS
void readFromGPS() {//reads gps serial and updates position information

  while (serialGPS.available())
  {
    gps.encode(serialGPS.read());
  }
  if (gps.time.isUpdated()) { //gets coords and simplify them to a positive cartesian plane
    myLat = String(gps.location.lat() - 3, 6);
    myLng = String((gps.location.lng() * -1) - 76, 6);

  }
  if (myLat != "-3.000000" && myLat != "0" && !gpsOK) {
    gpsOK = true;
    briteLoop(3);
    briteBlink(3);
  }
}

//controler
void control() {
  // wait for debounce time
  key = get_key(analogRead(0));    // convert into key press
  if (key != oldkey) {
    oldkey = key;
    if (key == 1 && !itsAlive) {
      runningExp++;
      if (runningExp > 4) {
        runningExp = 1;
      }
      briteBlink(runningExp);
      return;
    }
    if (key == 2 && itsAlive) {
      myFile.close();
      itsAlive = false;
      briteLoop(5);
      briteLED.sendColor(0, 0, 0);
      return;
    }

    if (key == 2 && !itsAlive) {
      gps.encode(serialGPS.read());
      char fileName[12] = "000000.TXT";

      if (gps.time.hour() < 10) {
        fileName[0] = '0';
        fileName[1] = String(gps.time.hour())[0];
      } else {
        fileName[0] = String(gps.time.hour())[0];
        fileName[1] = String(gps.time.hour())[1];
      }
      if (gps.time.minute() < 10) {
        fileName[2] = '0';
        fileName[3] = (String(gps.time.minute()))[0];
      } else {
        fileName[2] = (String(gps.time.minute()))[0];
        fileName[3] = (String(gps.time.minute()))[1];
      }
      if (gps.time.second() < 10) {
        fileName[4] = '0';
        fileName[5] = (String(gps.time.second()))[0];
      } else {
        fileName[4] = (String(gps.time.second()))[0];
        fileName[5] = (String(gps.time.second()))[1];
      }
      if (myFile.open(fileName, O_CREAT | O_EXCL | O_WRITE)) {
        itsAlive = true;
        desiredSpacing = expValues[0 + ((runningExp - 1) * 10)];
        minSpeed = expValues[1 + ((runningExp - 1) * 10)];
        circuit = expValues[2 + ((runningExp - 1) * 10)];
        wheels[0] = expValues[2 + ((runningExp - 1) * 10)];
        wheels[1] = expValues[3 + ((runningExp - 1) * 10)];
        wheels[2] = expValues[4 + ((runningExp - 1) * 10)];
        wheels[3] = expValues[5 + ((runningExp - 1) * 10)];
        alphaSpeed[0] = expValues[6 + ((runningExp - 1) * 10)];
        alphaSpeed[1] = expValues[7 + ((runningExp - 1) * 10)];
        alphaSpeed[2] = expValues[8 + ((runningExp - 1) * 10)];
        alphaSpeed[3] = expValues[9 + ((runningExp - 1) * 10)];
        Serial1.print("S" + String(desiredSpacing) + "|" + String(beaconInterval) + "|" + String(platoonInterval) + "|" + String(minSpeed, 2) + "|" + String(wheels[1], 2) + "|" + String(wheels[2], 2) + "|" + String(wheels[3], 2)+ "|"  + String(alphaSpeed[0], 2)+ "|"+String(alphaSpeed[1], 2)+ "|"+String(alphaSpeed[2], 2)+ "|"+String(alphaSpeed[3], 2) + "#");

        myFile.println("S" + String(desiredSpacing) + "|" + String(beaconInterval) + "|" + String(platoonInterval) + "|" + String(minSpeed, 2) + "|" + String(wheels[1], 2) + "|" + String(wheels[2], 2) + "|" + String(wheels[3], 2)+ "|" + String(alphaSpeed[0], 2)+ "|"+String(alphaSpeed[1], 2)+ "|"+String(alphaSpeed[2], 2)+ "|"+String(alphaSpeed[3], 2));
        briteLoop(5);
        briteLED.sendColor(0, 0, 512);
      } else {
        Serial.println("no creo nuevo archivo archivo " + String(fileName));
        briteLoop(2);
      }
    }
  }
  key = 0;
}


int get_key(unsigned int input)
{
  if (input > 600 && input < 900) {
    return 2;
  }
  if (input < 600) {
    return 1;
  }
  return 0;
}

void briteLoop(int n) {

  for (int i = 0; i < n; i++) {
    briteLED.sendColor(512, 0, 0);
    delay(250);
    briteLED.sendColor(0, 512, 0);
    delay(250);
    briteLED.sendColor(0, 0, 512);
    delay(250);
    briteLED.sendColor(0, 0, 0);
    delay(250);
  }
}

void  briteBlink(int n) {

  for (int i = 0; i < n; i++) {
    briteLED.sendColor(512, 512, 512);
    delay(500);
    briteLED.sendColor(0, 0, 0);
    delay(500);

  }
}
