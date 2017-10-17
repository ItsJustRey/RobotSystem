#include "ENVIRONMENT.h"
typedef int environment_T;
void ENVIRONMENT<environment_T>::prc_environment(){

	// UPDATE EACH ROBOT
	for (int i = 0; i < *(_numRobots); i++){


		if (robot_start_moving_port[i].read() == 1){


			// CHECK IF SERVER ALLOWED ROBOT TO MOVE
			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
			if (gridUpdate_port[i].read() == 1 && checkingBoundary[i] == true)
			{
				checkingBoundary[i] = false;		// finally received signal 
				r_cg_array[i] += 1;					// UPDATE GRIDS
				r_ng_array[i] += 1;					// UPDATE GRIDS
				r_x_array[i] = -1;					// START FROM BEGINNING OF GRID
			}
			else{
				r_cg_array[i] = r_cg_array[i];		// DONT UPDATE GRIDS
				r_ng_array[i] = r_ng_array[i];		// DONT UPDATE GRIDS
				r_x_array[i] = r_x_array[i];		// DONT UPDATE GRIDS
			}



			// CHECK IF ROBOT IS ABOUT TO CROSS BOUNDARY
			// Server(in) <-- ROBOT(inout) <-- Environment(out)
			if ((r_x_array[i] + r_speed_array[i]) >= GRID_WIDTH){

				// CROSSING
				checkingBoundary[i] = true;
				boundary_port[i].write(1);

			}
			else{
				// NOT CROSSING
				boundary_port[i].write(0);

				// ONLY UPDATE  X/Y WHEN ROBOT DOES NOT DETECT OBSTACLES
				if (detectedObstacle[i] == false){
					r_x_array[i] = r_x_array[i] + r_speed_array[i];
				}
				else{
					r_x_array[i] = r_x_array[i];
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


void ENVIRONMENT<environment_T>::prc_print_environment(){


	cout << endl << "~~~~~~~~~~~~~~~~~~~~~~~ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
	cout << endl << "===========================ROBOT ARRAY===========================" << endl;
	cout << "|RI\t|CG\t|NG\t|X\t|Y\t|BOUND\t|GRID\t|OBST\t|" << endl;
	cout << "=================================================================" << endl;
	for (int i = 0; i < *(_numRobots); i++){

		e_robot_array[i][0] = r_index_array[i];
		e_robot_array[i][1] = r_cg_array[i];
		e_robot_array[i][2] = r_ng_array[i];
		e_robot_array[i][3] = r_x_array[i];
		e_robot_array[i][4] = r_y_array[i];
		e_robot_array[i][5] = boundary_port[i].read();
		e_robot_array[i][6] = checkingBoundary[i];
		e_robot_array[i][7] = detectedObstacle[i];
		cout << "|";
		for (int j = 0; j < ROBOT_NUM_COLUMNS; j++){
			cout << e_robot_array[i][j] << "\t|";
		}
		cout << endl;
		/*cout << "Boundary Port" <<  << endl;
		cout << "Grid Update " <<  << endl;*/
	}
	cout << "=================================================================" << endl;


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


	cout << "`````````````````````````````````````````````````````````````````" << endl;
	cout << "`````````````````````````````````````````````````````````````````" << endl;
	cout << "`````````````````````````````````````````````````````````````````" << endl;
	cout << "`````````````````````````````````````````````````````````````````" << endl;

}
