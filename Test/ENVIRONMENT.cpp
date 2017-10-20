#include "ENVIRONMENT.h"
typedef int environment_T;
void ENVIRONMENT<environment_T>::prc_environment(){


	

	// UPDATE EACH ROBOT
	for (int i = 0; i < *(_numRobots); i++){


		if (robot_start_moving_port[i].read() == 1){


			///////////////
			// UPDATE ROBOT PATH
			for (int j = 0; j < 6; j++)
			{

				if (i ==0)
					block0_array[j] = block0_array_port_in[j].read();
				else if (i ==1)
					block1_array[j] = block1_array_port_in[j].read();
				else if (i ==2)
					block2_array[j] = block2_array_port_in[j].read();
			}
			///////////////////

			

			r_cg_array[i] = e_cg_array_port[i].read();
			r_ng_array[i] = e_ng_array_port[i].read();


			// CHECK IF SERVER ALLOWED ROBOT TO MOVE
			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
			if (gridUpdate_port[i].read() == 1 && checkingBoundary[i] == true && abs(r_ng_array[i]-r_cg_array[i])== 1 )
			{
				checkingBoundary[i] = false;		// finally received signal 
				r_cg_array[i] = e_cg_array_port[i].read();					// UPDATE GRIDS
				r_ng_array[i] = e_ng_array_port[i].read();				// UPDATE GRIDS
				r_x_array[i] = 0;					// START FROM BEGINNING OF GRID
				speedControl[i] = 0; // RESET
			}
			else{
				r_cg_array[i] = e_cg_array_port[i].read();		// DONT UPDATE GRIDS
				r_ng_array[i] = r_ng_array[i];		// DONT UPDATE GRIDS
				r_x_array[i] = r_x_array[i];		// DONT UPDATE GRIDS
				speedControl[i] = 0; // RESET
			}



			
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			if (gridUpdate_port[i].read() == 1 && checkingBoundary[i] == true && abs(r_ng_array[i] - r_cg_array[i]) !=1)
			{
	
				checkingBoundary[i] = false;		// finally received signal 
				r_cg_array[i] = e_cg_array_port[i].read();					// UPDATE GRIDS
				r_ng_array[i] = e_ng_array_port[i].read();				// UPDATE GRIDS
				r_y_array[i] = 0;					// START FROM BEGINNING OF GRID
				speedControl[i] = 0; // RESET
			}
			else{
				r_cg_array[i] = e_cg_array_port[i].read();		// DONT UPDATE GRIDS
				r_ng_array[i] = r_ng_array[i];		// DONT UPDATE GRIDS
				r_y_array[i] = r_y_array[i];		// DONT UPDATE GRIDS
				speedControl[i] = 0; // RESET
			}


			if (abs(r_ng_array[i] - r_cg_array[i]) == 1){

				// CHECK IF ROBOT IS ABOUT TO CROSS BOUNDARY
				// Server(in) <-- ROBOT(inout) <-- Environment(out)
				if ((r_x_array[i] + r_speed_array[i]) >= GRID_WIDTH){

					// CROSSING
					checkingBoundary[i] = true;
					boundary_port[i].write(1);
					speedControl[i] = -1; // SLOW DOWN
				}
				else{
					// NOT CROSSING
					boundary_port[i].write(0);
					cout << "SPEED BEFORE WRITE " << r_speed_array[i];


					cout << "SPEED AFTER WRITE " << r_speed_array[i];

					// ONLY UPDATE  X/Y WHEN ROBOT DOES NOT DETECT OBSTACLES
					if (detectedObstacle[i] == false){
						r_x_array[i] = r_x_array[i] + r_speed_array[i];
					}
					else{
						r_x_array[i] = r_x_array[i];
					}
				}


			}
			else{

				if ((r_y_array[i] + r_speed_array[i]) >= GRID_HEIGHT){

					// CROSSING
					checkingBoundary[i] = true;
					boundary_port[i].write(1);
					speedControl[i] = -1; // SLOW DOWN
				}
				else{
					// NOT CROSSING
					boundary_port[i].write(0);

					// ONLY UPDATE  X/Y WHEN ROBOT DOES NOT DETECT OBSTACLES
					if (detectedObstacle[i] == false)// && IsY==1)
					{
						r_y_array[i] = r_y_array[i] + r_speed_array[i];
					}
					else{
						r_y_array[i] = r_y_array[i];
					}
				}
			}


			

			// CHECK IF ROBOT IS NEAR OBSTACLE
			// Server(in) <-- ROBOT(inout) <-- Environment(out)
			for (int j = 0; j < *(_numObstacles); j++){

				// DETECTED OBSTACLE
				if (r_cg_array[i] == o_cg_array[j]
					&& ((r_x_array[i] + (r_speed_array[i])) >= o_x_array[j])
					&& (r_y_array[i] == o_y_array[j])){

					cout << "ROBOT " << i << " DETECTED OBSTACLE " << j << endl;
					detectedObstacle[i] = true;
					obstacle_port[i].write(1);
				}

				// DID NOT DETECT OBSTACLE
				else{
					obstacle_port[i].write(0);
				}
			}
		}
	}


}


void ENVIRONMENT<environment_T>::prc_robot0_obstacle_detected(){

	cout << " PRC_ROBOT0_OBSTACLE_DETECTED " << endl;

	// EITHER MOVE OBJECT OR MOVE ROBOT IDK???

	// PROB MOVE ROBOT



}


void ENVIRONMENT<environment_T>::prc_robot1_obstacle_detected(){

	cout << " PRC_ROBOT1_OBSTACLE_DETECTED " << endl;

	// EITHER MOVE OBJECT OR MOVE ROBOT IDK???

	// PROB MOVE ROBOT



}

void ENVIRONMENT<environment_T>::prc_nodes(){
	//sc_int<8> node0_priority[E_NUM_ROBOTS];
	//sc_int<8> node4_priority[E_NUM_ROBOTS];
	//sc_int<8> node20_priority[E_NUM_ROBOTS];
	//sc_int<8> node22_priority[E_NUM_ROBOTS];
	//sc_int<8> node24_priority[E_NUM_ROBOTS];
	cout << " prc_nodes() " << endl;
	// FOR EACH ROBOT
	for (int i = 0; i < *(_numRobots); i++){

		if (robot_start_moving_port[i].read() == 1){

			// ITERATE THROUGH THIS ROBOTS PATH
			for (int j = 0; j < 6; j++){

				//ROBOT 0
				if (i == 0){
					
				   if (block0_array[j] == 4){
						node4_robots_in_path[0]=1;
					}

					else if (block0_array[j] == 20){
						node20_robots_in_path[0] = 1;
					}
					else if (block0_array[j] == 22){
						node22_robots_in_path[0] = 1;
					}
					else if (block0_array[j] == 24){
						node24_robots_in_path[0] = 1;
					}
				}

				//ROBOT 1
				else if (i == 1){
					

					if (block1_array[j] == 4){
						node4_robots_in_path[1] = 1;
					}

					else if (block1_array[j] == 20){
						node20_robots_in_path[1] = 1;
					}
					else if (block1_array[j] == 22){
						node22_robots_in_path[1] = 1;
					}
					else if (block1_array[j] == 24){
						node24_robots_in_path[1] = 1;
					}
				}

				//ROBOT 2
				else if (i == 2){
					

					if (block2_array[j] == 4){
						node4_robots_in_path[2] = 1;
					}

					else if (block2_array[j] == 20){
						node20_robots_in_path[2] = 1;
					}
					else if (block2_array[j] == 22){
						node22_robots_in_path[2] = 1;
					}
					else if (block2_array[j] == 24){
						node24_robots_in_path[2] = 1;
					}
				}
				
			}

		}

	}
	// PRIORITIZE ROBOTS

	// FOR EACH NODE IN "nodexx_robots_in_path[]"
	for (int i = 0; i < E_NUM_ROBOTS; i++){

		int robotIndex;
		// CHECK IF THIS
		if (node22_robots_in_path[i] == 1){
			
			int x = abs(22 - r_cg_array[1]);
			int y = abs(22 - r_cg_array[2]);

			if (x < y){
				speedControl[2] = 2;
				speedControl[1] = -1;

			}

		}

	
	}

}
	
void ENVIRONMENT<environment_T>::prc_speed_control(){


	for (int i = 0; i < E_NUM_ROBOTS; i++){

		// -1 = slow down            
		if (speedControl[i] == -1){

			if (r_speed_array[i] - 1 < 0){
				r_speed_array[i] = 0;
			}
			else{
				r_speed_array[i] = r_speed_array[i] - 1;
			}
			
		}

		//  0 = reset
		else if (speedControl[i] == 0){
			if (i == 0){
				r_speed_array[i] = *(_r0_speed);
			}
			else if (i == 1){
				r_speed_array[i] = *(_r1_speed);
			}
			else if (i == 2){
				r_speed_array[i] = *(_r2_speed);
			}



		}

		//  1 = normal
		else if (speedControl[i] == 1){

			r_speed_array[i] = r_speed_array[i];

		}


		// 2 = speed up
		else if (speedControl[i] == 2){
			if (r_speed_array[i] + 1 < 5){
				r_speed_array[i] = 5;
			}
			else{
				r_speed_array[i] = r_speed_array[i] + 1;
			}
			

		}
	}
}




void ENVIRONMENT<environment_T>::prc_print_environment(){

	//next_trigger(1.0,SC_SEC);
	//next_trigger(1.0, SC_SEC);

	cout << endl << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
	cout << endl << "===============================ROBOT ARRAY=======================================" << endl;
	cout << "|RI\t|CG\t|NG\t|X\t|Y\t|SPEED\t|MOVING\t|BOUND\t|GRID\t|OBST\t|" << endl;
	cout << "=================================================================================" << endl;
	for (int i = 0; i < *(_numRobots); i++){

		e_robot_array[i][0] = r_index_array[i];
		e_robot_array[i][1] = r_cg_array[i];
		e_robot_array[i][2] = r_ng_array[i];
		e_robot_array[i][3] = r_x_array[i];
		e_robot_array[i][4] = r_y_array[i];
		e_robot_array[i][5] = r_speed_array[i];
		e_robot_array[i][6] = robot_start_moving_port[i].read();
		e_robot_array[i][7] = checkingBoundary[i];
		e_robot_array[i][8] = gridUpdate_port[i].read();
		e_robot_array[i][9] = detectedObstacle[i];
		cout << "|";
		for (int j = 0; j < ROBOT_NUM_COLUMNS; j++){
			cout << e_robot_array[i][j];
			if (j == 5){
				cout << " m/s\t|";
			}
			else{
				cout << "\t|";
			}
		}
		cout << endl;
		/*cout << "Boundary Port" <<  << endl;
		cout << "Grid Update " <<  << endl;*/
	}
	cout << "=================================================================================" << endl;

	// PRINT ROBOT PATH
	// UPDATE EACH ROBOT
	for (int i = 0; i < *(_numRobots); i++){
		if (robot_start_moving_port[i].read() == 1){
			cout << "ENVIRONMENT ROBOT " << i << " PATH: ";
			for (int j = 0; j < 6; j++)
			{

				if (i == 0)
					cout << "(" << block0_array[j] << ") --> ";
				else if (i == 1)
					cout << "(" << block1_array[j] << ") --> ";
				else if (i == 2)
					cout << "(" << block2_array[j] << ") --> ";
			}
			cout << endl;
		}
	}
	///////////////////



	cout << endl << "==============OBSTACLE ARRAY=============" << endl;
	cout << "|OI\t|CG\t|NG\t|X\t|Y\t|" << endl;
	cout << "=========================================" << endl;
	for (int i = 0; i < E_NUM_OBSTACLES; i++){

		e_obstacle_array[i][0] = o_index_array[i];
		e_obstacle_array[i][1] = o_cg_array[i];
		e_obstacle_array[i][2] = o_ng_array[i];
		e_obstacle_array[i][3] = o_x_array[i];
		e_obstacle_array[i][4] = o_y_array[i];
		cout << "|";
		for (int j = 0; j < OBSTACLE_NUM_COLUMNS; j++){
			cout << e_obstacle_array[i][j] << "\t|";
		}
		cout << endl;
	}
	cout << "=========================================" << endl;


	cout << endl << "==============NODES ARRAY=============" << endl;
	cout << "|NI\t|RO\t|R1\t|R2\t" << endl;
	cout << "=========================================" << endl;
	
	cout << "4\t";
	for (int i = 0; i < E_NUM_ROBOTS; i++){

		cout << node4_robots_in_path[i] << "\t|";
	}
	cout << endl;
	cout << "20\t";
	for (int i = 0; i < E_NUM_ROBOTS; i++){

		cout << node20_robots_in_path[i] << "\t|";
	}
	cout << endl;
	cout << "22\t";
	for (int i = 0; i < E_NUM_ROBOTS; i++){

		cout << node22_robots_in_path[i] << "\t|";
	}
	cout << endl;
	cout << "24\t";
	for (int i = 0; i < E_NUM_ROBOTS; i++){

		cout << node24_robots_in_path[i] << "\t|";
	}

	cout << endl;
	cout << "=========================================" << endl;


	cout << "`````````````````````````````````````````````````````````````````" << endl;
	cout << "`````````````````````````````````````````````````````````````````" << endl;
	cout << "`````````````````````````````````````````````````````````````````" << endl;
	cout << "`````````````````````````````````````````````````````````````````" << endl;

}



void ENVIRONMENT<environment_T>::prc_speed_data(){
	
	


	/*robot0_file << r_speed_array[0] << " ";
	robot1_file << r_speed_array[1] << " ";
	robot2_file << r_speed_array[2] << " ";*/




}