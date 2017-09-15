

/*
  This program controls the Followers on the bicycle field experiments.
  The code was develope with five main branches and its been assemble on a single sketch, those branches are

  --Speedometer code, responsible for the speed camculations and sensor, it uses a single interrupt and some timers
  --Handlebars HCI motor controller, this code controls the speed and direction os the motor to comunicate the rider acceleration instructions
  --Xbee comunication module: responscible for sending ans recieving data, it will use AT mode and strings to send data and recieve instructions
  --GPS reading: the sole proporse its to update GPS readings from the GPS module once a second
  --Control Formula
  Hardware and software developed at HCI Lab - ICESI university -  Cali, Colombia, by Juan Manuel Salamanca and Daniel Vinasco
*/
//------------------------LIBRARIES------------------------\\

//GPS
#include <SoftwareSerial.h> //viertual serial for the GPS
#include <TinyGPS++.h> //Helps with the handling of the GPS
#include <math.h>
//megabrite
#include <TinyBrite.h>

//------------------------VARIABLES------------------------\\

//Debbugging
unsigned long time3; //keep track for USB serial COM


//Speedometer
unsigned long time1; // Control for debouncing the sensor and tracking speed
double   mySpeed;// The actual speed calculated each interruption
int sensorPIN = 2; // this must be one of the interruptables 0,1,2,3,7
double circuit = 1.52;// wheel diameter in cm to calculate covered distance vs time on each lap
double minSpeed = 2.75;

//Handlebar Motor Variables
int E2 = 6;    // Speed Control - aslo the pin number
int M2 = 7;    // Direction Control - also the pin number

//Xbee Com
String beaconMsg;
unsigned long time4;
unsigned long timerBeaconLeader;
double distanceBuffer = 1;

//GPS
SoftwareSerial serialGPS(9, 10); // Creates virtual serial for the GPS (RX, TX) pins,
TinyGPSPlus gps; //GPS object
double myLat; //My latitud
double myLng; //My longitud
unsigned long time2;
bool gpsOK = false;

//control Formula

// 1. General variables
double totalDistance = 0; // Total distance
double lastAccelerationPlatoon = 0; // Last acceleration calculated with CACC
double localLeaderAcceleration = 0; // Acceleration of leader
double localLeaderSpeed = 0; // // Speed of leader

//2 . Platoon's parameters (CACC)
double minDist;// distance to front node
double  alpha1 = 0.5;
double alpha2 = 0.5;
double alpha3 = 0.3;
double alpha4 = 0.1;
double alpha5 = 0.04;
double alphaLag = 0.8; // para aterrisar la aceleracion
double length_vehicle_front = 2; //ancho de la bici
double desiredSpacing = 10;//variable programable via botones
int beaconInterval = 1000; // How often a beacon is sent
int platoonInterval = 1000;// how often the desire acceleration its calculated
// Platoon Size its 4

int bikeID = 1; // bikes 1 throw 3 are fallowers, and 0 the leader
double bikeAcc[3];
double bikeLat[3];
double bikeLng[3];
double bikeSpeed[3];

double bikeAccL = 0;
double bikeLatL = 0;
double bikeLngL = 0;
double bikeDistL = 0;
double bikeSpeedL = 0;

int rel_speed_front = 0; //velocidad relativa con respecto a la bici del frente
int closestBike = bikeID - 1;
unsigned long time5;
const double Pi = 3.141593;
double alphaSpeed = 0;

//setup
bool setupLoop = true;
bool setupLoop2 = true;

//megabrite
#define clockpin  A3
#define latchpin  A5
#define datapin   A4
TinyBrite briteLED(1, TINYBRITE_AUTOUPDATE_ENABLE);
unsigned long time00 = millis();
unsigned long time01 = 0; //millis();
int redLed = 0;
int greenLed = 0;
int blueLed = 0;
int    redLedTemp = 0;
int    greenLedTemp = 0;
int    blueLedTemp = 0;
int ledBlinkerTimer = 0;
int ledBlinkerFreq = 4;
//------------------------SETUP------------------------\\

void setup() {
  //debugging

  Serial.begin(9600); //USB serial
  time3 = millis();

  //speedometer Setup
  pinMode(sensorPIN, INPUT);
  attachInterrupt(1, speedSensor, RISING);//interrupt for pin 2
  time1 = millis();

  //Handlebar Motor
  pinMode(M2, OUTPUT);
  pinMode(E2, OUTPUT);

  //Xbee Com
  Serial1.begin(9600); //XBee serial1
  time4 = millis();

  //GPS
  serialGPS.begin(9600); //GPS SoftSerial
  time2 = millis();

  //control Formula
  time5 = millis();

  // create the megabrite chain, as a global
  briteLED.setup(datapin, clockpin, latchpin);

  //create timer interrupt
  cli();//stop interrupts


  //set timer1 interrupt at 1Hz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = 1562;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
}
//------------------------Program Loop------------------------\\

void loop() {
  //setup
  /*
    while (setupLoop2) {
    readFromGPS();
    setMeUp();
    Serial.println("loop");
    }*/
  //speedometer
  // Cleans the speed sensor buffer on stop
  speedometer();

  //GPS

  //xbee
  // Reads the beacon from leader and paired followers. Assign variables from bike ahead (speed, distance)
  readBeacons();

  // verifies that GPS signal and config package are received. Enables CACC
  if (gpsOK && !setupLoop) {
    // read GPS, calculate distance to bike ahead, invoke CACC, invoke haptic motor, brightLights
    readFromGPS();
    flagBiker(); //CACC + LEDS
    // Sends beacon
    sendBeacon();
  } else {
    // reads GPS to validate coordinates before running the experiment
    readFromGPS();
  }


}
//------------------------Functions------------------------\\

//debugging
//flagBiker leds and motor
void flagBiker() {
  if (millis() - time5 > platoonInterval) { //this loop will start after sat signal its confirm at the intervals
    time5 = millis();
    if (mySpeed > 0) {

      // 1. PREPARE


      double nodeFrontAcceleration;
      // 2. START PLATOON

      double  spacing_error = -minDist + length_vehicle_front + desiredSpacing;
      //d. Calculate (Acceleration desired) A_des /// esta es la formula que calcula la aceleracion deserada
      double A_des = alpha1 * bikeAcc[closestBike] + alpha2 * bikeAccL - alpha3 * rel_speed_front - alpha4 * (mySpeed - bikeSpeedL) - alpha5 * spacing_error;

      //e. Calculate desired acceleration adding a delay //acelerecacion realista
      lastAccelerationPlatoon = (alphaLag * A_des) + ((1 - alphaLag) * lastAccelerationPlatoon);

      // define the blinking frequency
      if (lastAccelerationPlatoon < alphaSpeed * 1 && lastAccelerationPlatoon > -alphaSpeed) {
        ledBlinkerFreq = 10;
        return;

      }

      if (lastAccelerationPlatoon < alphaSpeed * 1.2 && lastAccelerationPlatoon > -alphaSpeed * 1.2) {
        ledBlinkerFreq = 8;
        return;

      }

      if (lastAccelerationPlatoon < alphaSpeed * 1.3 && lastAccelerationPlatoon > -alphaSpeed * 1.3) {
        ledBlinkerFreq = 6;
        return;

      }

      if (lastAccelerationPlatoon < alphaSpeed * 1.4 && lastAccelerationPlatoon > -alphaSpeed * 1.4) {
        ledBlinkerFreq = 4;
        return;

      }

      if (lastAccelerationPlatoon < alphaSpeed * 1.5 && lastAccelerationPlatoon > -alphaSpeed * 1.5) {
        ledBlinkerFreq = 3;
        return;

      }
      if (lastAccelerationPlatoon < alphaSpeed * 1.7 && lastAccelerationPlatoon > -alphaSpeed * 1.7) {
        ledBlinkerFreq = 2;
        return;

      }
      if (lastAccelerationPlatoon < alphaSpeed * 2 && lastAccelerationPlatoon > -alphaSpeed * 2) {
        ledBlinkerFreq = 1;
        return;

      }

    }

  }
  if (mySpeed > 0 && lastAccelerationPlatoon < -alphaSpeed) {
    motorControl(1)   ;
    // ledControl(512, 0, 0);
    redLedTemp = 512;
    greenLedTemp = 0;
    blueLedTemp = 0;
  }
  if (mySpeed > 0 && lastAccelerationPlatoon > alphaSpeed) {
    motorControl(2) ;
    // ledControl(0, 512, 0);
    redLedTemp = 0;
    greenLedTemp = 512;
    blueLedTemp = 0;
  }
  if (mySpeed == 0) {
    motorControl(3) ;
    // ledControl(0, 0, 512);
    redLedTemp = 0;
    greenLedTemp = 0;
    blueLedTemp = 0;
    lastAccelerationPlatoon = 0;
  }

  if (mySpeed > 0 && lastAccelerationPlatoon <= alphaSpeed && lastAccelerationPlatoon >= -alphaSpeed) {
    motorControl(2) ;
    // ledControl(0, 512, 0);
    redLedTemp = 0;
    greenLedTemp = 0;
    blueLedTemp = 512;
  }
}
//speedometer
void speedSensor() {// this fuction its executeded each time the sensor interrupts arduino

  if (millis() - time1 > 100) {
    mySpeed = (circuit * 1000) / ((millis() - time1));
    if (millis() - time1 > 3000)
      mySpeed = 0;
    time1 = millis();
  }
}
void speedometer() {
  if (millis() - time1 > 2000) //checks if the wheel hasn't move for 2 seconds and assumes it has stopt, resert speed to 0
    mySpeed = 0;

}

//Handlebar Motor

void motorControl(int dirMotor) // controla direccion del motor donde 1 es adelante, 2 atras y 3 parado
{
  if (dirMotor == 1) { //motor arranca adelante
    digitalWrite(M2, HIGH);
    analogWrite (E2, 150);

  }
  if (dirMotor == 2) { //motor atras
    digitalWrite(M2, LOW);
    analogWrite (E2, 150);

  }
  if (dirMotor == 3) {
    digitalWrite(M2, LOW);
    analogWrite (E2, 0);
  }
}

//Xbee Com
void sendBeacon()// pruebas de envio de datos por serial un char a la vez
{
  if (millis() - time4 > (beaconInterval)) {
    time4 = millis();
    while (serialGPS.available())
    {
      gps.encode(serialGPS.read());
    }
    Serial1.print(String(bikeID) + "|" + String(myLat, 6) + "|" + String(myLng, 6) + "|" + String(mySpeed, 2)  + "|" + String(gps.time.hour() - 5) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "|" + String(lastAccelerationPlatoon, 2) + "|" + String(minDist, 2) + "|" + "#") ; //leader also sends its timer millis() after the
    Serial1.flush();
  }
}
void readBeacons() {

  int beaconFrom;
  int index1;
  long timerTemp;
  String linea;

  if (Serial1.available()) {
    linea = Serial1.readStringUntil('#');

    Serial.println(linea);
    beaconFrom = linea.charAt(0);
    linea.remove(0, 2);
    Serial.println(minDist);
    if (beaconFrom == '0' || beaconFrom == '1' || beaconFrom == '2' || beaconFrom == '3') {
      int bc = beaconFrom - '0';

      index1 = linea.indexOf('|');
      bikeLat[bc] = (linea.substring(0, index1)).toFloat();
      linea.remove(0, index1 + 1);

      index1 = linea.indexOf('|');
      bikeLng[bc] = (linea.substring(0, index1)).toFloat();

      linea.remove(0, index1 + 1);

      index1 = linea.indexOf('|');
      bikeSpeed[bc] = (linea.substring(0, index1)).toFloat();

    }
    if (beaconFrom == 'S') {
      //setup

      index1 = linea.indexOf('|');
      desiredSpacing = (linea.substring(0, index1)).toFloat();
      linea.remove(0, index1 + 1);
      index1 = linea.indexOf('|');
      beaconInterval = (linea.substring(0, index1)).toFloat();

      linea.remove(0, index1 + 1);

      index1 = linea.indexOf('|');
      platoonInterval = (linea.substring(0, index1)).toFloat();

      linea.remove(0, index1 + 1);

      index1 = linea.indexOf('|');
      minSpeed = (linea.substring(0, index1)).toFloat();
      linea.remove(0, index1 + 1);
      double temp = 0;
      for (int i = 0; i < 3; i++) {
        index1 = linea.indexOf('|');

        temp = (linea.substring(0, index1)).toFloat();
        linea.remove(0, index1 + 1);
        if (i + 1 == bikeID) {
          circuit = temp;
        }
      }

      for (int i = 0; i < 3; i++) {
        index1 = linea.indexOf('|');

        temp = (linea.substring(0, index1)).toFloat();
        linea.remove(0, index1 + 1);
        if (i + 1 == bikeID) {
          alphaSpeed = temp;
        }
      }
      briteLoop(5);

    }
  }

}
//GPS
void readFromGPS() {//reads gps serial and updates position information


  while (serialGPS.available())
  {
    gps.encode(serialGPS.read());

  }
  if (gps.time.isUpdated()) { //gets coords and simplys them to a positive cartesian plane - Collin's law
    myLat = gps.location.lat() - 3;
    myLng = (gps.location.lng() * -1) - 76 ;
    //myLat = gps.location.lat();
    //myLng = gps.location.lng();

    float dY;
    float dX;

    dX = (bikeLat[closestBike] - myLat) * 111194.9;
    dY = 111194.9 * (bikeLng[closestBike] - myLng) * cos(radians((bikeLat[closestBike] + myLat) / 2));


    minDist = sqrt(pow(dX, 2) + pow(dY, 2));
    //  Serial.println("Distancia " + String(minDist));
  }

  if (myLat != -3 && myLat != 0 && !gpsOK) {

    gpsOK = true;
    briteLoop(3);
    briteBlink(3);
  }
}


void ledControl(int red, int green, int blue) {

  if (red == redLed && green == greenLed && blue == blueLed) {
  } else {
    redLed = red;
    greenLed = green;
    blueLed = blue;
    briteLED.sendColor(redLed, greenLed, blueLed);
  }

}

void briteLoop(int n) {
  setupLoop = true;
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
  setupLoop = false;
}

void  briteBlink(int n) {
  setupLoop = true;
  for (int i = 0; i < n; i++) {
    briteLED.sendColor(512, 512, 512);
    delay(500);
    briteLED.sendColor(0, 0, 0);
    delay(500);

  }
  setupLoop = false;
}

ISR(TIMER1_COMPA_vect) { // timer interrupt for blinker
  if (!setupLoop) {
    if (ledBlinkerTimer == ledBlinkerFreq) {
      ledControl(0, 0, 0);
      ledBlinkerTimer = 0;
    } else {
      ledControl(redLedTemp, greenLedTemp, blueLedTemp);

      ledBlinkerTimer++;
    }
  }


}

