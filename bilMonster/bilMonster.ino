/*
Drive forward and turn left or right when border is detected
  - Only reflectionsensor 0 and 5 are used.
*/
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <PLab_ZumoMotors.h>
 
#define LED 13
 
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1800 // 
  
// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
 
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

PLab_ZumoMotors plab_Motors;

#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];
 
ZumoReflectanceSensorArray sensors;


 
void setup()
{
   sensors.init();
   button.waitForButton();
}

void loop()
{
  sensors.read(sensor_values);

  if (sensor_values[0] < QTR_THRESHOLD)
  {
     plab_Motors.backward(REVERSE_SPEED, 10);
     plab_Motors.turnRight(300,90);
  }
  else if (sensor_values[5] < QTR_THRESHOLD)
  {
     plab_Motors.backward(REVERSE_SPEED, 10);
     plab_Motors.turnLeft(300,90);
  }
  else if (sensor_values[2] < QTR_THRESHOLD){
    plab_Motors.backward(200,10);
    plab_Motors.turnLeft(300,180);
  }
  else if (sensor_values[4] < QTR_THRESHOLD){
    plab_Motors.backward(10,200);
    plab_Motors.turnRight(300,180);
  }
  else
  {
    // otherwise, go straight
    motors.setSpeeds(400, 300);
    
    
    
  }
}



