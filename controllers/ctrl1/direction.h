#include <webots/DifferentialWheels.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <cstdlib>
#include <cmath>

#define ON              1
#define OFF             0
#define NR_DIST_SENSORS 8
#define TIME_STEP       64

#define MAX_VAL       	  3900
#define DS_TRASHOLD   		0.0
#define DS_TRASHOLD_BIG   0.5
#define ROTATE180_TRASHOLD 2.5

#define ROTATE_VAL 260
#define ROTATE_SPEED 400

#define DIR_CLK 1
#define DIR_CONTCLK (-1)

#define WH_C_10   	   100//270 //0.09
#define WH_C_45        100//270 //0.07
#define WH_C_90        100//270 //0.07

#define WR_C_10   	   150//390 //0.09
#define WR_C_45        150//390 //0.07
#define WR_C_90        150//390 //0.07

//Turnaround Module Constants
#define TA_STATE_DONOTHING 0
#define TA_STATE_BACKOFF   1
#define TA_STATE_ROTATE    2

#define CTOI_TRIGGER  100
#define CTOI_DISTANCE 200
#define ITOC_DISTANCE 500

using namespace webots;

#define BASE_SPEED    300

enum Machine_States{
	STATE_FOLLOW_CORIDOR,
	STATE_MOVE_TO_INTERSECT_CENTER,
	STATE_MOVE_TO_CORIDOR,
	STATE_INTERSECTION,
	STATE_ROTATE,
	STATE_STOP
};

class VirtualEncoder{
public:
	void reset(DifferentialWheels &difw);
	long get_left(DifferentialWheels &difw);
	long get_right(DifferentialWheels &difw);

private:
	long _left,_right;
	long _left_old,_right_old;
};


class DIRECTION{
public:

	DIRECTION();
	~DIRECTION();

	void reset_modifiers();

	//Behaviors
	void update_ds();
	void wall_hugger();
	void wall_repeller();
	bool turnarround();

	//State select and apply
	void run();
	void switch_state(Machine_States next_state);

private:
	//States
	Machine_States curr_state;

	unsigned long long counttime;
	int speed_left,speed_right;
	int direction_rotate;
	
	
	//SENSORS
	double wh_delta;
	double wr_delta;
	DistanceSensor **dist_sensors;
	double total_delta_left, total_delta_right;
	VirtualEncoder ve_aux;
	VirtualEncoder ve_node_to_node;
	VirtualEncoder ve_rotate;

	//ACTUATORS
	double ds_left_10,ds_left_45,ds_left_90;
	double ds_right_10, ds_right_45,ds_right_90;

	DifferentialWheels diff_wheels;
};
