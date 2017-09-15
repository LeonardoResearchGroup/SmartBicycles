
//------------------------LIBRARIES------------------------\\


#include <SoftwareSerial.h> //viertual serial for the GPS

#include <math.h>

//Joystick >600 desaselera <300 acelera 500 Quieto


//------------------------VARIABLES------------------------\\



//Speedometer
unsigned long time1; // Control for debouncing the sensor and tracking speed
double   mySpeed;// The actual speed calculated each interruption
int sensorPin = 4; // this must be one of the interruptables 0,1,2,3,7
double circuit = 1.52;// wheel diameter in cm to calculate covered distance vs time on each lap
double oldSpeed;

//Handlebar Motor Variables
int motor1 = 3;    // Speed Control - aslo the pin number
int motor2 = 0;    // Direction Control - also the pin number


//BT
SoftwareSerial serialBT(1, 2); // Creates virtual serial for the GPS (RX, TX) pins,
const double Pi = 3.141593;



//------------------------SETUP------------------------\\

void setup() {
  //debugging

  serialBT.begin(9600); //BT serial

  //speedometer Setup
  pinMode(A2, INPUT);
  time1 = millis();
  oldSpeed = 0;

  //Handlebar Motor
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);

  digitalWrite(motor1, LOW);
  digitalWrite(motor2, HIGH);
  delay(4000);
  digitalWrite(motor2, LOW);
  digitalWrite(motor1, HIGH);
  delay(4000);
  digitalWrite(motor2, LOW);
  digitalWrite(motor1, LOW);
  delay(16000);

  digitalWrite(motor1, LOW);
  digitalWrite(motor2, HIGH);

}
//------------------------Program Loop------------------------\\

void loop() {


  readBeacons();
  speedSensor();
}
//------------------------Functions------------------------\\



void speedSensor() {// this fuction its executeded each time the sensor interrupts arduino
  int dataSensor = 0;


  dataSensor = analogRead(A2);


  if (dataSensor > 600) {
    mySpeed += 0.05;

  }
  if (dataSensor < 300) {
    mySpeed -= 0.05;

  }
  if (mySpeed < 0) {
    mySpeed = 0;
  }
  if (mySpeed > 20) {
    mySpeed = 20;
  }

  if (oldSpeed != mySpeed) {
    serialBT.print(String(mySpeed) + '#');
    oldSpeed = mySpeed;
  }
}



void readBeacons() {
  if (serialBT.available()) {
    String linea = serialBT.readStringUntil('#');
    if (linea == "-1") {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, LOW);
      digitalWrite(motor1, HIGH);
    }
    if (linea == "0") {
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, LOW);
    }
    if (linea == "1") {
      digitalWrite(motor2, LOW);
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, HIGH);
    }
  }


}



