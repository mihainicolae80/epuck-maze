// File:          ctrl1.cpp
// Date:          
// Description:   
// Author:        
// Modifications: 

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/LED.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/DifferentialWheels.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#include "map.h"
#include "direction.h"


// All the webots classes are defined in the "webots" namespace
using namespace webots;


#define TIME_STEP     64
#define ON            1
#define OFF           0
#define NR_DIST_SENSORS 8

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
// Note that this class derives Robot and so inherits all its functions
class ctrl1 : public Robot {
  
  // You may need to define your own functions or variables, like
  //  LED *led;
  
  public:
    
    // ctrl1 constructor
    ctrl1(): Robot() {
      
      int i;
      char ds_name[] = "ps0";
      
      //Set wheels speed
      diff_wheels.setSpeed(100,100);
      
      //Init distance sensors
      dist_sensors = new DistanceSensor* [NR_DIST_SENSORS];
      
      for(i = 0; i < NR_DIST_SENSORS; i++){
      
        dist_sensors[i] = new DistanceSensor(ds_name);
        dist_sensors[i]->enable(TIME_STEP);
        ds_name[2]++;
      }
      
    }

    // ctrl1 destructor
    virtual ~ctrl1() {
      
      int i;
      
      for(i = 0; i < NR_DIST_SENSORS; i++){
      
        delete dist_sensors[i];
      }
      
      delete[] dist_sensors;
    }
    
    void turn_when_front_wall(){
    
      int ds_front_left;
      int ds_front_right;
      
      ds_front_right = dist_sensors[7]->getValue();
      ds_front_left = dist_sensors[0]->getValue();
      
      std::cout<<"right="<<ds_front_right;
      std::cout<<" left="<<ds_front_left<<std::endl;
    
    }
    
    // User defined function for initializing and running
    // the ctrl1 class
    void run(){
      
      // Main loop
      do {
        
        direction.run();
        map.run();
        
        
      } while (step(64) != -1);
    }
  private:
  
  MAP map;
  DIRECTION direction;
  
  DifferentialWheels diff_wheels;
  DistanceSensor **dist_sensors;
};

// This is the main program of your controller.
// It creates an instance of your Robot subclass, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
  ctrl1* controller = new ctrl1();
  controller->run();
  delete controller;
  return 0;
}
