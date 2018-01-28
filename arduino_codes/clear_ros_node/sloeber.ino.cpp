#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2018-01-28 18:38:35

#include "Arduino.h"
#include <ros.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "Arduino.h"
#include <avr/wdt.h>
#include <Wire.h>
#include "headers/arduino_ros_interface.h"
#include "headers/vehicle.h"

void sendArduinoStatus(void) ;
void reserveDynamicMemory(void) ;
void setup() ;
bool checkIfItsTimeToInterfaceWithROS(void) ;
void receiveROSInputs(void) ;
void sendOutputsToROS(void) ;
void loop() ;

#include "clear_ros_node.ino"


#endif
