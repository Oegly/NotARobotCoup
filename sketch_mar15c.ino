/*
  Finn ein ven!
*/

#include <NewPing.h>
#include <NewServo.h>
#include <Pushbutton.h>
#include <PLab_ZumoMotors.h>
#include <ZumoMotors.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <PLabBTSerial.h>


const int servoPin = 6;
const int echoPin = A1;
const int ledPin = 3;
const int triggerPin = A0;
const int maxDistance = 40;

int distance;
int degreesServo = 0;
int degreesStep = 5;

// Definer inn/utgangspinnene som brukes for send (TX) og motta (RX) for bluetooth
// Define I/O ports used for transmit (TX) and receive (RX)
const int BT_RX = 1;  // Connect to RXD on Bluetooth unit
const int BT_TX = 2;  // Connect to TXD on Bluetooth unit

// Definer serieporten for kommunikasjon med bluetooth
// Define the serial port for communication with bluetooth
PLabBTSerial btSerial (BT_TX, BT_RX);

// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1800 // 
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     400 // 0 is stopped, 400 is full speed
#define TURN_SPEED        260
#define FORWARD_SPEED     400

#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
 
ZumoReflectanceSensorArray sensors;

Pushbutton button(ZUMO_BUTTON);
ZumoMotors motors;
PLab_ZumoMotors plab_Motors;
NewPing sonar(triggerPin, echoPin, maxDistance);
NewServo myServo; 

void setup() {
  sensors.init();
  Serial.begin(9600);
  pinMode(ledPin,OUTPUT);
  myServo.attach(servoPin); 
  myServo.write(46);
  button.waitForButton(); // start when button pressed
}

void stepServo() {
   degreesServo = degreesServo + degreesStep;
   if (degreesServo > 180) {
       degreesStep = -degreesStep;
       degreesServo = 180;
   } else if (degreesServo < 0) {
       degreesStep = -degreesStep;
       degreesServo = 0;
   } 
   myServo.write(degreesServo);
}

float sonarDistance() {
  // Gjør ett ping, og beregn avstanden
  //motors.setSpeeds(60, -60);
  unsigned int time = sonar.ping();
  float distance = sonar.convert_cm(time);
  Serial.println(distance);
  
  if (distance == 0.0) { // sonar gives zero when outside range
    // Turn off LED and just go forward
    digitalWrite(ledPin,LOW); 
   } else {
    digitalWrite(ledPin,HIGH);
   }
   
   return distance;
}

void avoidEdge() {
   sensors.read(sensor_values);
   if (sensor_values[0] < QTR_THRESHOLD) {
     plab_Motors.backward(REVERSE_SPEED, 10);
     plab_Motors.turnRight(TURN_SPEED,120);
     plab_Motors.forward(FORWARD_SPEED, 30);
   } else if (sensor_values[5] < QTR_THRESHOLD) {
     plab_Motors.backward(REVERSE_SPEED, 10);
     plab_Motors.turnLeft(TURN_SPEED,120);
     plab_Motors.forward(FORWARD_SPEED, 30);
   }
   /*else
   {
    // otherwise, go straight
     motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }*/
}

void findFriend() {
  if(sonarDistance() > 0) {
    hugFriend();
  } else {
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  }
}

void hugFriend() {
  motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  while(sensor_values[0] > QTR_THRESHOLD && sensor_values[5] > QTR_THRESHOLD) {
    sensors.read(sensor_values);
  }
  motors.setSpeeds(0, 0);
}

void loop() {
  avoidEdge();
  btSerial.println(sonarDistance());
  findFriend();
}
