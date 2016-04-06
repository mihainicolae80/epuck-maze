#include "direction.h"

DIRECTION::DIRECTION(){
	
	int i;
	char ds_name[] = "ps0";

	//Init distance sensors
	dist_sensors = new DistanceSensor* [NR_DIST_SENSORS];

	for(i = 0; i < NR_DIST_SENSORS; i++){
		dist_sensors[i] = new DistanceSensor(ds_name);
		dist_sensors[i]->enable(TIME_STEP);
		ds_name[2]++;
	}

	wh_delta  = 0;
	
	ds_left_10  = ds_left_45  = ds_left_90  = 0;
	ds_right_10 = ds_right_45 = ds_right_90 = 0;
	
	total_delta_left = total_delta_right = 0;
}

DIRECTION::~DIRECTION(){
	
	int i; 
	for(i = 0; i < NR_DIST_SENSORS; i++){

		delete dist_sensors[i];
	}
	delete[] dist_sensors;
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
	
}

void DIRECTION::wall_hugger(){
	
	wh_delta = 0;
	
	///TODO Aplica Log_2 pe readingurile de pe senzori
	
	if(ds_left_10 > DS_TRASHOLD || ds_left_45 > DS_TRASHOLD 
	   || ds_left_90 >	DS_TRASHOLD){
		   
			wh_delta = -WH_C_10 * ds_left_10
						- WH_C_45 * ds_left_45
						- WH_C_90 * ds_left_90;
	   }
	else
	if(ds_right_10 > DS_TRASHOLD || ds_right_45 > DS_TRASHOLD 
	   || ds_right_90 >	DS_TRASHOLD){
		   
			wh_delta = WH_C_10 * ds_left_10
						+ WH_C_45 * ds_left_45
						+ WH_C_90 * ds_left_90;
	   }	
}

void DIRECTION::run(){

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
	
	
	total_delta_right = -wh_delta;
	total_delta_left  =  wh_delta;
}

int DIRECTION::get_delta_left(){
	return total_delta_left;
}
int DIRECTION::get_delta_right(){
	return total_delta_right;
}
