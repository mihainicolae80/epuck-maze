#include "direction.h"

/*
		TODO
		-Distanta minima pana la cel mai apropiat nod neparcurs
		-Output-uri pt graf
		-Detectare daca un drum este deschis
		-deocamdata drumul "back este mereu deschis". verifica
		cum se comporta asa si daca nu e bine , foloseste senzorii
		de 180*
		-bypass pt move_to_corridor daca s-a rotit
		datorita lui 180Rotate_trigger
*/

int constrain(int x){

	 if(x > 1000)
		return  1000;
	 else
	 if(x < -1000)
	 	return -1000;

		return x;
}

//TODO verifica daca este ok
int get_rotate_direction(int curr, int goal){
	if(abs(curr - goal) == 2){
		//Irelevant
		return DIR_CLK;
	}
	else if(curr - goal == 0){
		return DIR_CLK;
	}
	else {
		if(curr == DIR_NORTH){
			if(goal == DIR_EAST){
				return DIR_CLK;
			}
			else{
				return DIR_CONTCLK;
			}
		}
		else if(curr == DIR_EAST){
			if(goal == DIR_SOUTH){
				return DIR_CLK;
			}
			else{
				return DIR_CONTCLK;
			}
		}
		else if(curr == DIR_SOUTH){
			if(goal == DIR_WEST){
				return DIR_CLK;
			}
			else{
				return DIR_CONTCLK;
			}
		}
		else if(curr == DIR_WEST){
			if(goal == DIR_NORTH){
				return DIR_CLK;
			}
			else{
				return DIR_CONTCLK;
			}
		}
	}

	std::cout << "Error: Invalid direction" << std::endl;
	return DIR_NORTH;
}
int get_rotate_times(int curr, int goal){
	if(abs(curr - goal) == 2){
		//Irelevant
		return ANGLE_180;
	}
	else if(curr - goal == 0){
		return ANGLE_0;
	}
	else {
		return ANGLE_90;
	}
}

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

	orientation = DIR_NORTH;
	old_orientation = DIR_NORTH;

	ve_node_to_node.reset(diff_wheels);

	int i;
	char ds_name[] = "ps0";
	char led_name[] = "led0";

	counttime = 0;
	x = 0;
	y = 0;

	//Init distance sensors
	dist_sensors = new DistanceSensor* [NR_DIST_SENSORS];
	led = new LED * [NB_LEDS];

	for(i = 0; i < NR_DIST_SENSORS; i++){
		dist_sensors[i] = new DistanceSensor(ds_name);
		dist_sensors[i]->enable(TIME_STEP);
		ds_name[2]++;
	}

	for(i = 0; i < NB_LEDS; i++){
		led[i] = new LED(led_name);
		led[i]->set(0);
		led_name[3]++;
	}

	wh_delta  = 0;
	wr_delta  = 0;

	ds_left_10  = ds_left_45  = ds_left_90  = 0;
	ds_right_10 = ds_right_45 = ds_right_90 = 0;

	total_delta_left = total_delta_right = 0;

	switch_state(STATE_FOLLOW_CORIDOR); //STATE_FOLLOW_CORIDOR;
	direction_rotate = DIR_CLK;
	angle_rotate = ANGLE_90;
}

DIRECTION::~DIRECTION(){

	int i;
	for(i = 0; i < NR_DIST_SENSORS; i++){

		delete dist_sensors[i];
	}
	delete[] dist_sensors;

	for(i = 0; i < NB_LEDS; i++){

		delete led[i];
	}
	delete[] led;
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

	if(ds_left_45 > DS_TRASHOLD_BIG2
	// || ds_left_45 > DS_TRASHOLD_BIG
	// || ds_left_90 > DS_TRASHOLD_BIG
){
		wall_on_left = true;
	}

	double maxleft = ds_left_10 > ds_left_45 ? ds_left_10 : ds_left_45;
	maxleft = maxleft > ds_left_90 ? maxleft : ds_left_90;

	double maxright = ds_right_10 > ds_right_45 ? ds_right_10 : ds_right_45;
	maxright = maxright > ds_right_90 ? maxright : ds_right_90;

	//std::cout<<"maxleft="<<maxleft<<std::endl;
	//std::cout<<"maxright="<<maxright<<std::endl;

 if(ds_right_10 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_C_10 * ds_right_10;
 if(ds_right_45 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_C_45 * ds_right_45;
 if(ds_right_90 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_C_90 * ds_right_90;

	 if(ds_right_45 > DS_TRASHOLD_BIG2
	//  || ds_right_45 > DS_TRASHOLD_BIG
  //  || ds_right_90 > DS_TRASHOLD_BIG
 ){
	 		wall_on_right = true;
	 	}

		//TODO decomenteaza

		//Curba sau intersectie
		if(!wall_on_left || !wall_on_right){
		//if(abs(maxleft-maxright) > 0.4){
				wr_delta = 0;

				//if(!wall_on_left) std::cout << "no_wall_left" << std::endl;
				//if(!wall_on_right) std::cout << "no_wall_right" << std::endl;

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


void DIRECTION::wall_repeller_noenc(){

	wr_delta = 0;

	if(ds_left_10 > DS_TRASHOLD_BIG)
		wr_delta += WR_C_10 * ds_left_10;
	if(ds_left_45 > DS_TRASHOLD_BIG)
		wr_delta += WR_C_45 * ds_left_45;
	if(ds_left_90 > DS_TRASHOLD_BIG)
		wr_delta += WR_C_90 * ds_left_90;

 if(ds_right_10 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_C_10 * ds_right_10;
 if(ds_right_45 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_C_45 * ds_right_45;
 if(ds_right_90 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_C_90 * ds_right_90;

}

void DIRECTION::wall_repeller_aux(){

	wr_delta = 0;

	if(ds_left_10 > DS_TRASHOLD_BIG)
		wr_delta -= WR_CAUX_10 * ds_left_10;
	if(ds_left_45 > DS_TRASHOLD_BIG)
		wr_delta -= WR_CAUX_45 * ds_left_45;
	if(ds_left_90 > DS_TRASHOLD_BIG)
		wr_delta -= WR_CAUX_90 * ds_left_90;


	double maxleft = ds_left_10 > ds_left_45 ? ds_left_10 : ds_left_45;
	maxleft = maxleft > ds_left_90 ? maxleft : ds_left_90;

 if(ds_right_10 > DS_TRASHOLD_BIG)
	 wr_delta += WR_CAUX_10 * ds_right_10;
 if(ds_right_45 > DS_TRASHOLD_BIG)
	 wr_delta += WR_CAUX_45 * ds_right_45;
 if(ds_right_90 > DS_TRASHOLD_BIG)
	 wr_delta += WR_CAUX_90 * ds_right_90;
}

void DIRECTION::wall_repeller_aux2(){

	wr_delta = 0;

	if(ds_left_10 > DS_TRASHOLD_BIG)
		wr_delta += WR_CAUX2_10 * ds_left_10;
	if(ds_left_45 > DS_TRASHOLD_BIG)
		wr_delta += WR_CAUX2_45 * ds_left_45;
	if(ds_left_90 > DS_TRASHOLD_BIG)
		wr_delta += WR_CAUX2_90 * ds_left_90;


 if(ds_right_10 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_CAUX2_10 * ds_right_10;
 if(ds_right_45 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_CAUX2_45 * ds_right_45;
 if(ds_right_90 > DS_TRASHOLD_BIG)
	 wr_delta -= WR_CAUX2_90 * ds_right_90;

}

void DIRECTION::run(){

	reset_modifiers();

	//State machine
 	if(     curr_state == STATE_FOLLOW_CORIDOR){

		update_ds();
		wall_repeller();

		//std::cout << "left=" << ve_aux.get_left(diff_wheels) << std::endl;
		//std::cout << "right=" << ve_aux.get_right(diff_wheels) << std::endl;

		//Daca exista un perete in fata
		//TODO Decomenteaza
		if(ds_left_10 >= DS_ROTATE180_TRIGGER
			|| ds_right_10 >= DS_ROTATE180_TRIGGER){
				switch_state(STATE_INTERSECTION);
			}

		//Compute total modifier
		total_delta_right =  - wr_delta;
		total_delta_left  =  + wr_delta;
	}
	else if(curr_state == STATE_MOVE_TO_INTERSECT_CENTER){

		// update_ds();
		//
		// wall_repeller_aux();
		//
		// total_delta_right =  - wr_delta;
		// total_delta_left  =  + wr_delta;


		total_delta_right =  0;
		total_delta_left  =  0;

		if(ve_aux.get_left(diff_wheels) > CTOI_DISTANCE
		   || ve_aux.get_right(diff_wheels) > CTOI_DISTANCE){

			 switch_state(STATE_INTERSECTION);

		 }
	}
	else if(curr_state == STATE_MOVE_TO_CORIDOR){

		//  update_ds();
		//  wall_repeller_noenc();
		//  total_delta_right =  - wr_delta;
 	// 	 total_delta_left  =  + wr_delta;

	total_delta_left  = 0;
	total_delta_right = 0;

		//
		 //wall_repeller_aux2();
		//
		// total_delta_right =  - wr_delta;
		// total_delta_left  =  + wr_delta;




		//
		// std::cout << "abs_left=" << diff_wheels.getLeftEncoder() << std::endl;
		// std::cout << "abs_right=" << diff_wheels.getRightEncoder() << std::endl;
		// std::cout << "left=" << ve_aux.get_left(diff_wheels) << std::endl;
		// std::cout << "right=" << ve_aux.get_right(diff_wheels) << std::endl;


		if(ve_aux.get_left(diff_wheels) > ITOC_DISTANCE
		   || ve_aux.get_right(diff_wheels) > ITOC_DISTANCE ){
			 switch_state(STATE_FOLLOW_CORIDOR);
		 }
	}
	else if(curr_state == STATE_ROTATE){

		if(times_rotate != 0){
			total_delta_left = direction_rotate*-ROTATE_SPEED -BASE_SPEED;
			total_delta_right = direction_rotate*ROTATE_SPEED -BASE_SPEED;

		}
		else{
			switch_state(STATE_MOVE_TO_CORIDOR);
		}


		if((abs(ve_rotate.get_right(diff_wheels)) >= ROTATE_VAL
			  && times_rotate  == 1)
		||((abs(ve_rotate.get_right(diff_wheels)) >= ROTATE_VAL_180
		    && times_rotate  == 2))){
				switch_state(STATE_MOVE_TO_CORIDOR);
		}
	}
	else if(curr_state == STATE_STOP){
		total_delta_left  = -BASE_SPEED;
		total_delta_right = -BASE_SPEED;
		//TODO Mai simplu daca se cal viteza direct?
	}




	/*

		-Adauga un nod nou in graf sau o muchie
		(nod nou, daca intersectia nu a mai fost vizitate
		si muchie daca intersectia a mai fost vizitate)
		-Determina in ce directie poate sa
		mearga.
		-Alege o directie

	*/
	else if(curr_state == STATE_INTERSECTION){

		//Nu se deplaseaza
		total_delta_left  = -BASE_SPEED;
		total_delta_right = -BASE_SPEED;



		//Calculeaza distanta
		int avg_dist = (ve_node_to_node.get_left(diff_wheels)
									 + ve_node_to_node.get_right(diff_wheels))
									 /2;


		//Actualizeaza coordonatele
		if(orientation == DIR_NORTH){
			y -= avg_dist;
		}
		else if(orientation == DIR_SOUTH){
			y += avg_dist;
		}
		else if(orientation == DIR_EAST){
			x += avg_dist;
		}
		else if(orientation == DIR_WEST){
			x -= avg_dist;
		}

		//TODO Detectare daca un drum este deschis
		//Analizeaza pozitia curenta
		//Ia date de la senzori

		bool opennorth, opensouth, openeast, openwest;

		identify_walls(opennorth, opensouth, openeast, openwest);

		//Starea wall-urilor
		std::cout<<"north="<<opennorth<<std::endl;
		std::cout<<"south="<<opensouth<<std::endl;
		std::cout<<"east="<<openeast<<std::endl;
		std::cout<<"west="<<openwest<<std::endl;


		//TODO Fix
		curr_node = graph.on_intersection(x, y, orientation, opennorth, opensouth, openeast, openwest);

		std::cout<<"cnode_north="<<curr_node->open_on_dir[DIR_NORTH]<<std::endl;
		std::cout<<"cnode_south="<<curr_node->open_on_dir[DIR_SOUTH]<<std::endl;
		std::cout<<"cnode_east="<<curr_node->open_on_dir[DIR_EAST]<<std::endl;
		std::cout<<"cnode_west="<<curr_node->open_on_dir[DIR_WEST]<<std::endl;

		std::cout<<"cnode_north_viz="<<curr_node->visited_on_dir[DIR_NORTH]<<std::endl;
		std::cout<<"cnode_south_viz="<<curr_node->visited_on_dir[DIR_SOUTH]<<std::endl;
		std::cout<<"cnode_east_viz="<<curr_node->visited_on_dir[DIR_EAST]<<std::endl;
		std::cout<<"cnode_west_viz="<<curr_node->visited_on_dir[DIR_WEST]<<std::endl;

		old_orientation = orientation;

		//Pregateste o decizie dupa semnalare
		if(curr_node->open_on_dir[DIR_NORTH]
		&& !curr_node->visited_on_dir[DIR_NORTH]){
			direction_rotate = get_rotate_direction(orientation,DIR_NORTH);
			times_rotate = get_rotate_times(orientation,DIR_NORTH);
			orientation = DIR_NORTH;
		}
		else
		if(curr_node->open_on_dir[DIR_SOUTH]
		&& !curr_node->visited_on_dir[DIR_SOUTH]){
			direction_rotate = get_rotate_direction(orientation,DIR_SOUTH);
			times_rotate = get_rotate_times(orientation,DIR_SOUTH);
			orientation = DIR_SOUTH;
		}
		else
		if(curr_node->open_on_dir[DIR_EAST]
		&& !curr_node->visited_on_dir[DIR_EAST]){
			direction_rotate = get_rotate_direction(orientation,DIR_EAST);
			times_rotate = get_rotate_times(orientation,DIR_EAST);
			orientation = DIR_EAST;
		}
		else
		if(curr_node->open_on_dir[DIR_WEST]
		&& !curr_node->visited_on_dir[DIR_WEST]){
			direction_rotate = get_rotate_direction(orientation,DIR_WEST);
			times_rotate = get_rotate_times(orientation,DIR_WEST);
			orientation = DIR_WEST;
		}
		else if(curr_node->open_on_dir[DIR_NORTH]){
			direction_rotate = get_rotate_direction(orientation,DIR_NORTH);
			times_rotate = get_rotate_times(orientation,DIR_NORTH);
			orientation = DIR_NORTH;
		}
		else
		if(curr_node->open_on_dir[DIR_SOUTH]){
			direction_rotate = get_rotate_direction(orientation,DIR_SOUTH);
			times_rotate = get_rotate_times(orientation,DIR_SOUTH);
			orientation = DIR_SOUTH;
		}
		else
		if(curr_node->open_on_dir[DIR_EAST]){
			direction_rotate = get_rotate_direction(orientation,DIR_EAST);
			times_rotate = get_rotate_times(orientation,DIR_EAST);
			orientation = DIR_EAST;
		}
		else
		if(curr_node->open_on_dir[DIR_WEST]){
			direction_rotate = get_rotate_direction(orientation,DIR_WEST);
			times_rotate = get_rotate_times(orientation,DIR_WEST);
			orientation = DIR_WEST;
		}
		else{
			//TODO daca nodul curent a fost explorat in toate
			//directile, se merge la cel mai apropiat nod care
			//nu a fost explorat complet

			std::cout<<"No place to go\n";
			switch_state(STATE_STOP);
			return;
		}


		//std::cout<<"Willgo="<<orientation;

		//Semnalizeaza directile in care poate merge
		switch_state(STATE_SIGNAL);
	}
	/*Semnalizeaza */
	else if(curr_state == STATE_SIGNAL){
		/*
			 Pentru determinarea directiilor accesibile fizic
			 si a celor vizitate:
			 curr_node->open_on_dir[DIR_NORTH]
			 curr_node->open_on_dir[DIR_SOUTH]
			 curr_node->open_on_dir[DIR_EAST]
			 curr_node->open_on_dir[DIR_WEST]
			 curr_node->visited_on_dir[DIR_NORTH]
			 curr_node->visited_on_dir[DIR_SOUTH]
			 curr_node->visited_on_dir[DIR_EAST]
			 curr_node->visited_on_dir[DIR_WEST]
		*/

		if(curr_node != NULL){

			//Determina valorile relative lalabirint

			dir abs_north,abs_south,abs_east,abs_west;

			if(old_orientation == DIR_NORTH){
				abs_north = DIR_NORTH;
				abs_south = DIR_SOUTH;
				abs_east  = DIR_EAST;
				abs_west  = DIR_WEST;
			}
			else if(old_orientation == DIR_SOUTH){
				abs_north = DIR_SOUTH;
				abs_south = DIR_NORTH;
				abs_east  = DIR_WEST;
				abs_west  = DIR_EAST;
			}
			else if(old_orientation == DIR_EAST){
				abs_north = DIR_EAST;
				abs_south = DIR_WEST;
				abs_east  = DIR_SOUTH;
				abs_west  = DIR_NORTH;
			}
			else if(old_orientation == DIR_WEST){
				abs_north = DIR_WEST;
				abs_south = DIR_EAST;
				abs_east  = DIR_NORTH;
				abs_west  = DIR_SOUTH;
			}

			// if(curr_node->open_on_dir[abs_north]
			// && !curr_node->visited_on_dir[abs_north]){
			//
			// 	if(leds_on) led[0]->set(1);
			// 	else 			  led[0]->set(0);
			// 	//led[0]->set(1);
			// }
			// else
			if(curr_node->open_on_dir[abs_north]){
				led[0]->set(1);
			}

			// if(curr_node->open_on_dir[abs_south]
			// && !curr_node->visited_on_dir[abs_south]){
			// 	if(leds_on) led[4]->set(1);
			// 	else 			  led[4]->set(0);
			// 	//led[4]->set(1);
			// }
			// else
			if(curr_node->open_on_dir[abs_south]){
				led[4]->set(1);
			}

			// if(curr_node->open_on_dir[abs_east]
			// && !curr_node->visited_on_dir[abs_east]){
			// 	if(leds_on) led[2]->set(1);
			// 	else 			  led[2]->set(0);
			// 	//led[2]->set(1);
			// }
			// else
			if(curr_node->open_on_dir[abs_east]){
				led[2]->set(1);
			}


			// if(curr_node->open_on_dir[abs_west]
			// && !curr_node->visited_on_dir[abs_west]){
			// 	if(leds_on) led[6]->set(1);
			// 	else 			  led[6]->set(0);
			// 	//led[6]->set(1);
			// }
			// else
			if(curr_node->open_on_dir[abs_west]){
				led[6]->set(1);
			}

		}
		else{
			std::cout << "ERR: STATE_SIGNAL curr_node == NULL" << std::endl;
		}

		//Daca a stat in aceasta stare 3 secunde
		if(time(NULL) - timer >= 1){

			//std::cout << "Time to switch\n" << std::endl;

			set_all_leds_off();//TODO~mod

			switch_state(STATE_ROTATE);
		}
		else{
			//std::cout << "time_diff="<< time(NULL) - timer << std::endl;
		}

		if(time(NULL) - timer_blink_led >= 1 ){
			leds_on = !leds_on;
			timer_blink_led = time(NULL);
			//std::cout<<"switching leds\n";
		}

}

	//Set speeds
	speed_right = constrain(BASE_SPEED + total_delta_right);
	speed_left  = constrain(BASE_SPEED + total_delta_left);
	diff_wheels.setSpeed(speed_left,speed_right);
}

void DIRECTION::set_all_leds_off(){//TODO~mod
	for(int i = 0; i < NB_LEDS; i++){
		led[i]->set(0);
	}
}

void DIRECTION::identify_walls(bool &opennorth,bool &opensouth,bool &openeast, bool &openwest){
	update_ds();

	bool front = true,back = true,left = true,right = true;

	if(ds_left_10 > DS_TRASHOLD_BIG
	|| ds_right_10 > DS_TRASHOLD_BIG){
		front = false;
	}

	if(ds_right_90 > DS_TRASHOLD_BIG){
		right = false;
	}

	if(ds_left_90 > DS_TRASHOLD_BIG){
		left  = false;
	}

	//Maybe yes, maybe not
	back = true;

	if(orientation == DIR_NORTH){
		opennorth = front;
		openeast  = right;
		openwest  = left;
		opensouth = back;
	}
	else
	if(orientation == DIR_SOUTH){
		opennorth = back;
		openeast  = left;
		openwest  = right;
		opensouth = front;
	}
	else
	if(orientation == DIR_EAST){
		opennorth = left;
		openeast  = front;
		openwest  = back;
		opensouth = right;
	}
	else
	if(orientation == DIR_WEST){
		opennorth = right;
		openeast  = back;
		openwest  = front;
		opensouth = left;
	}
}

void DIRECTION::switch_state(Machine_States next_state){
	curr_state = next_state;
	ve_aux.reset(diff_wheels);

	if(next_state ==STATE_FOLLOW_CORIDOR){
		//std::cout<<"next_state=STATE_FOLLOW_CORIDOR\n";
	}
	else if(next_state ==STATE_MOVE_TO_INTERSECT_CENTER){
		//std::cout<<"next_state=STATE_MOVE_TO_INTERSECT_CENTER\n";
	}
	else if(next_state == STATE_MOVE_TO_CORIDOR){
		//Reset encodere virtuale
		ve_node_to_node.reset(diff_wheels);

		//std::cout<<"next_state=STATE_MOVE_TO_CORIDOR\n";
	}
	else if(next_state == STATE_INTERSECTION){
		//std::cout<<"next_state=STATE_INTERSECTION\n";
	}
	else if(next_state == STATE_SIGNAL){
		//Reseteaza timer
		timer = time(NULL);
		timer_blink_led = time(NULL);
	}
	else if(next_state == STATE_ROTATE){
		ve_rotate.reset(diff_wheels);
		//std::cout<<"next_state=STATE_ROTATE\n";
	}
	else if(next_state == STATE_STOP){
		//std::cout<<"next_state=STATE_STOP\n";
	}
}
