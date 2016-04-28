#include "direction.h"


void VirtualEncoder::reset(DifferentialWheels &difw){
	_left  = _left_old  = difw.getLeftEncoder();
	_right = _right_old = difw.getRightEncoder();
}

long VirtualEncoder::get_left(DifferentialWheels &difw){

	//Error
	if(_left - _left_old > 300)
		_left = _left_old;

	return difw.getLeftEncoder() - _left;
}

long VirtualEncoder::get_right(DifferentialWheels &difw){

	//Error
	if(_right - _right_old > 300)
		_right = _right_old;

	return difw.getRightEncoder() - _right;
}

DIRECTION::DIRECTION(){

	//Set wheels speed
	diff_wheels.setSpeed(0,0);
	diff_wheels.enableEncoders(TIME_STEP);
  diff_wheels.setEncoders(0,0);

	ve_node_to_node.reset(diff_wheels);

	int i;
	char ds_name[] = "ps0";

	counttime = 0;

	//Init distance sensors
	dist_sensors = new DistanceSensor* [NR_DIST_SENSORS];

	for(i = 0; i < NR_DIST_SENSORS; i++){
		dist_sensors[i] = new DistanceSensor(ds_name);
		dist_sensors[i]->enable(TIME_STEP);
		ds_name[2]++;
	}

	wh_delta  = 0;
	wr_delta  = 0;

	ds_left_10  = ds_left_45  = ds_left_90  = 0;
	ds_right_10 = ds_right_45 = ds_right_90 = 0;

	total_delta_left = total_delta_right = 0;

	switch_state(STATE_FOLLOW_CORIDOR); //STATE_FOLLOW_CORIDOR;
	direction_rotate = DIR_CLK;
	// TODO add rotate times
}

DIRECTION::~DIRECTION(){

	int i;
	for(i = 0; i < NR_DIST_SENSORS; i++){

		delete dist_sensors[i];
	}
	delete[] dist_sensors;
}

void DIRECTION::reset_modifiers(){

	wh_delta = wr_delta = 0;
}

void DIRECTION::update_ds(){

	//Right
	ds_right_10 = dist_sensors[0] -> getValue();
	ds_right_45 = dist_sensors[1] -> getValue();
	ds_right_90 = dist_sensors[2] -> getValue();
	//Left
	ds_left_10 = dist_sensors[7] -> getValue();
	ds_left_45 = dist_sensors[6] -> getValue();
	ds_left_90 = dist_sensors[5] -> getValue();

	//Liniarizeaza valorile
	ds_right_10 = log(ds_right_10) - 5.0;
	ds_right_45 = log(ds_right_45) - 5.0;
	ds_right_90 = log(ds_right_90) - 5.0;

	ds_left_10 = log(ds_left_10) - 5.0;
	ds_left_45 = log(ds_left_45) - 5.0;
	ds_left_90 = log(ds_left_90) - 5.0;
}

void DIRECTION::wall_hugger(){

	wh_delta = 0;

		 if(ds_left_10 > DS_TRASHOLD)
			 wh_delta -= WH_C_10 * ds_left_10;
		 if(ds_left_45 > DS_TRASHOLD)
			 wh_delta -= WH_C_45 * ds_left_45;
		 if(ds_left_90 > DS_TRASHOLD)
			 wh_delta -= WH_C_90 * ds_left_90;

		//std::cout << "Going LEFT\n";


		if(ds_right_10 > DS_TRASHOLD)
			wh_delta += WH_C_10 * ds_right_10;
		if(ds_right_45 > DS_TRASHOLD)
			wh_delta += WH_C_45 * ds_right_45;
		if(ds_right_90 > DS_TRASHOLD)
			wh_delta += WH_C_90 * ds_right_90;

		//std::cout << "Going RIGHT\n";

}


void DIRECTION::wall_repeller(){

	wr_delta = 0;
	bool wall_on_left = false;
	bool wall_on_right = false;

	if(ds_left_10 > DS_TRASHOLD_BIG)
		wr_delta += WR_C_10 * ds_left_10;
	if(ds_left_45 > DS_TRASHOLD_BIG)
		wr_delta += WR_C_45 * ds_left_45;
	if(ds_left_90 > DS_TRASHOLD_BIG)
		wr_delta += WR_C_90 * ds_left_90;

	if(ds_left_10 > DS_TRASHOLD_BIG
	|| ds_left_45 > DS_TRASHOLD_BIG
	|| ds_left_90 > DS_TRASHOLD_BIG){
		wall_on_left = true;
	}

 if(ds_right_10 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_C_10 * ds_right_10;
 if(ds_right_45 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_C_45 * ds_right_45;
 if(ds_right_90 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_C_90 * ds_right_90;

	 if(ds_right_10 > DS_TRASHOLD_BIG
	 || ds_right_45 > DS_TRASHOLD_BIG
   || ds_right_90 > DS_TRASHOLD_BIG){
	 		wall_on_right = true;
	 	}

		//Curba sau intersectie
		if(!wall_on_left || !wall_on_right){
				wr_delta = 0;

				if(ve_aux.get_left(diff_wheels) > CTOI_TRIGGER
					 && ve_aux.get_right(diff_wheels) > CTOI_TRIGGER
					){
						 switch_state(STATE_MOVE_TO_INTERSECT_CENTER);
					}
		}
		//Daca merge pe un hol
		else{
			//reseteaza encoderul auxiliar
			ve_aux.reset(diff_wheels);
		}
}

bool DIRECTION::turnarround(){

	static int state = TA_STATE_DONOTHING;

	if(state == TA_STATE_DONOTHING
	&& ds_left_10 > ROTATE180_TRASHOLD
	&& ds_right_10 > ROTATE180_TRASHOLD ){
		state = TA_STATE_BACKOFF;
	}

	if(state == TA_STATE_BACKOFF){

		std::cout<<"In backoff state\n";
		//Retine val de pe encodere
		//suprascrie viteza cu una negativa si egala pe ambele roti
		//Cand (val_noua_encodere - val_veche_end > const)
		//state = ROTATE
	}
	else
	if(state == TA_STATE_ROTATE){
		//retine val de pe encodere
		//seteaza valori egale dar de sens opus pe roti
		//Cand (val_noua_encodere - val_veche_end > const)
		//state == DONOTHING
	}

	return false;
}

void DIRECTION::run(){

	/*
	//Notify is a sensors detects an object
	if(ds_left_10 > DS_TRASHOLD)
		std::cout<<"ds_left_10="<<ds_left_10<<std::endl;
	if(ds_left_45 > DS_TRASHOLD)
		std::cout<<"ds_left_45="<<ds_left_45<<std::endl;
	if(ds_left_90 > DS_TRASHOLD)
		std::cout<<"ds_left_90="<<ds_left_90<<std::endl;
	if(ds_right_10 > DS_TRASHOLD)
		std::cout<<"ds_right_10="<<ds_right_10<<std::endl;
	if(ds_right_45 > DS_TRASHOLD)
		std::cout<<"ds_right_45="<<ds_right_45<<std::endl;
	if(ds_right_90 > DS_TRASHOLD)
		std::cout<<"ds_right_90="<<ds_right_90<<std::endl;
	*/


	reset_modifiers();

	//State machine
	if(curr_state == STATE_FOLLOW_CORIDOR){

		std::cout << "state=follow_cor" << std::endl;

		update_ds();
		wall_repeller();

		std::cout << "left=" << ve_aux.get_left(diff_wheels) << std::endl;
		std::cout << "right=" << ve_aux.get_right(diff_wheels) << std::endl;

		//Compute total modifier
		total_delta_right =  - wr_delta;
		total_delta_left  =  + wr_delta;
	}
	else if(curr_state == STATE_MOVE_TO_INTERSECT_CENTER){
		std::cout << "state=blind_to_center" << std::endl;
		total_delta_left  = 0;
		total_delta_right = 0;

		if(ve_aux.get_left(diff_wheels) > CTOI_DISTANCE
		   || ve_aux.get_right(diff_wheels) > CTOI_DISTANCE){

			 switch_state(STATE_ROTATE);
			 ve_rotate.reset(diff_wheels);
		 }
	}
	else if(curr_state == STATE_ROTATE){
		std::cout << "state=rotate" << std::endl;

		total_delta_left = direction_rotate*-ROTATE_SPEED -BASE_SPEED;
		total_delta_right = direction_rotate*ROTATE_SPEED -BASE_SPEED;

		if( abs(ve_rotate.get_right(diff_wheels)) > ROTATE_VAL ){
				switch_state(STATE_MOVE_TO_CORIDOR);
		}
	}
	else if(curr_state == STATE_STOP){
		std::cout << "state=stopped" << std::endl;
		total_delta_left  = -BASE_SPEED;
		total_delta_right = -BASE_SPEED;
		//TODO Mai simplu daca se cal viteza direct?
	}
	else if(curr_state == STATE_MOVE_TO_CORIDOR){
		std::cout << "state=blind_to_coridor" << std::endl;
		total_delta_left  = 0;
		total_delta_right = 0;

		std::cout << "abs_left=" << diff_wheels.getLeftEncoder() << std::endl;
		std::cout << "abs_right=" << diff_wheels.getRightEncoder() << std::endl;
		std::cout << "left=" << ve_aux.get_left(diff_wheels) << std::endl;
		std::cout << "right=" << ve_aux.get_right(diff_wheels) << std::endl;


		if(ve_aux.get_left(diff_wheels) > ITOC_DISTANCE
		   || ve_aux.get_right(diff_wheels) > ITOC_DISTANCE ){
			 switch_state(STATE_FOLLOW_CORIDOR);
		 }
	}


	//Set speeds
	//TODO Constrangere pt valori 0-1000
	speed_right = BASE_SPEED + total_delta_right;
	speed_left  = BASE_SPEED + total_delta_left;
	diff_wheels.setSpeed(speed_left,speed_right);
}



void DIRECTION::switch_state(Machine_States next_state){
	curr_state = next_state;
	ve_aux.reset(diff_wheels);
}
