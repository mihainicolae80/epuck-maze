#include <webots/DifferentialWheels.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <cmath>

#define ON            1
#define OFF           0
#define NR_DIST_SENSORS 8
#define TIME_STEP     64

#define MAX_VAL       	  3900
#define DS_TRASHOLD   		0.0
#define DS_TRASHOLD_BIG   0.5
#define ROTATE180_TRASHOLD 2.5

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
#define CTOI_DISTANCE 400

using namespace webots;

#define BASE_SPEED    300

enum Machine_States{
	STATE_FOLLOW_CORIDOR,
	STATE_MOVE_TO_INTERSECT_CENTER,
	STATE_INTERSECTION,
	STATE_ROTATE,
	STATE_STOP
};

class VirtualEncoder{
public:
	void reset_at_values(long  left,long right);
	long get_left(long left);
	long get_right(long right);

private:
	long _left,_right;
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

private:
	//States
	Machine_States curr_state;

	unsigned long long counttime;
	int speed_left,speed_right;

	//SENSORS
	double wh_delta;
	double wr_delta;
	DistanceSensor **dist_sensors;
	double total_delta_left, total_delta_right;
	VirtualEncoder ve_atintersection;
	VirtualEncoder ve_to_it_center;
	VirtualEncoder ve_rotate;

	//ACTUATORS
	double ds_left_10,ds_left_45,ds_left_90;
	double ds_right_10, ds_right_45,ds_right_90;

	DifferentialWheels diff_wheels;
};
