#include <webots/DifferentialWheels.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp> // TODO~mod
#include <iostream>
#include <cstdlib>
#include <cmath>
#include "graph.h"
#include <ctime>

#define ON              1
#define OFF             0
#define NR_DIST_SENSORS 8
#define TIME_STEP       64

#define MAX_VAL       	  3900
#define DS_TRASHOLD   		0.0
#define DS_TRASHOLD_BIG   1.1
#define DS_ROTATE180_TRIGGER 1.5

#define ROTATE_VAL 250
#define ROTATE_VAL_180 600
#define ROTATE_SPEED 400

#define DIR_CLK    (-1)
#define DIR_CONTCLK 1

#define WH_C_10   	   100//270 //0.09
#define WH_C_45        100//270 //0.07
#define WH_C_90        100//270 //0.07

#define WR_C_10   	   60//150//390 //0.09
#define WR_C_45        60//150//390 //0.07
#define WR_C_90        60//150//390 //0.07

//Turnaround Module Constants
#define TA_STATE_DONOTHING 0
#define TA_STATE_BACKOFF   1
#define TA_STATE_ROTATE    2

#define CTOI_TRIGGER  50
#define CTOI_DISTANCE 210 //200
#define ITOC_DISTANCE 380 //500

#define ANGLE_0   0
#define ANGLE_90  1
#define ANGLE_180 2

#define NB_LEDS 8 // TODO~mod

using namespace webots;

#define BASE_SPEED    500//300

int constrain(int x);
int get_rotate_direction(int curr, int goal);
int get_rotate_times(int curr, int goal);

enum Machine_States{
	STATE_FOLLOW_CORIDOR,
	STATE_MOVE_TO_INTERSECT_CENTER,
	STATE_MOVE_TO_CORIDOR,
	STATE_INTERSECTION,
	STATE_SIGNAL,
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

	// LED behavior
	void set_all_leds_off();//TODO~mod

	//Behaviors
	void update_ds();
	void identify_walls(bool &opennorth,bool &opensouth,
											bool &openeast, bool &openwest);
	void wall_hugger();
	void wall_repeller();
	bool turnarround();

	//State select and apply
	void run();
	void switch_state(Machine_States next_state);

private:
	//timing
	time_t timer,timer_blink_led;
  bool leds_on;

	//Graph
	Graph graph;
	int x, y;
	dir orientation;
	dir old_orientation;
	Node *curr_node;
	//States
	Machine_States curr_state;

	unsigned long long counttime;
	int speed_left,speed_right;
	int direction_rotate;
	int times_rotate;
	int angle_rotate;


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

	LED **led;//TODO~mod
	DifferentialWheels diff_wheels;
};
