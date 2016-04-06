#include <webots/DifferentialWheels.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>

#define ON            1
#define OFF           0
#define NR_DIST_SENSORS 8
#define TIME_STEP     64

#define MAX_VAL       3900
#define DS_TRASHOLD   150

#define WH_C_10   	   0.09
#define WH_C_45        0.07
#define WH_C_90        0.07

using namespace webots;



class DIRECTION{
public:

	DIRECTION();
	~DIRECTION();

	void update_ds();
	
	void wall_hugger();
	void run();
	
	int get_delta_left();
	int get_delta_right();

private:	
	//SENSORS
	int wh_delta;
	DistanceSensor **dist_sensors;
	int total_delta_left, total_delta_right;
	
	//ACTUATORS
	int ds_left_10,ds_left_45,ds_left_90;
	int ds_right_10, ds_right_45,ds_right_90;
};