// File:          ctrl1.cpp
// Date:          
// Description:   
// Author:        
// Modifications: 

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <iostream>

#include "map.h"
#include "direction.h"


// All the webots classes are defined in the "webots" namespace
using namespace webots;


#define TIME_STEP     64


///TODO Fix senori pe o parte (nu schimba viteza pe motoare)

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
// Note that this class derives Robot and so inherits all its functions
class ctrl1 : public Robot {
  
  // You may need to define your own functions or variables, like
  //  LED *led;
  
  public:
    
    // ctrl1 constructor
    ctrl1(): Robot() {
      
      //Set wheels speed
      diff_wheels.setSpeed(0,0);
    }

    // ctrl1 destructor
    virtual ~ctrl1() {
      
      
    }
    
    
    // User defined function for initializing and running
    // the ctrl1 class
    void run(){
      int speed_left,speed_right;
      // Main loop
      do {
        //Read Sensors
        direction.update_ds();
        
        //Calculate actuator modifiers
        direction.wall_hugger();
        direction.run();
        
        
        //Set speeds
        speed_right = 400 + direction.get_delta_right();
        speed_left  = 400 + direction.get_delta_left();
        diff_wheels.setSpeed(speed_left,speed_right);
        
      } while (step(64) != -1);
    }
  private:
  
  MAP map;
  DIRECTION direction;
  
  DifferentialWheels diff_wheels;
  
};


int main(int argc, char **argv)
{
  ctrl1* controller = new ctrl1();
  controller->run();
  delete controller;
  return 0;
}
