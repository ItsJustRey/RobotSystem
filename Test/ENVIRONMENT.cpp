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
				
				//ROBOT 0
				if (i == 0)
				{
					int currentGridIndex = 0;
					for (int x = 0; x < 8; x++){
						if (robot0_grids[x] = r_cg_array[i]){
							currentGridIndex = x;
							cout << "CURRENT GRID INDEX OF ROBOT 0 IS " << currentGridIndex << endl;
							break;
						}
					}

					r_cg_array[i] = robot0_grids[currentGridIndex];		// UPDATE GRIDS
					r_ng_array[i] = robot0_grids[currentGridIndex+1];		// UPDATE GRIDS
					r_x_array[i] = -1;										// START FROM BEGINNING OF GRID
				}
				if (i == 1){

					int currentGridIndex = 0;
					for (int x = 0; x < 8; x++){
						if (robot1_grids[x] = r_cg_array[i]){
							cout << "CURRENT GRID INDEX OF ROBOT 1 IS " << currentGridIndex << endl;
							currentGridIndex = x;
							break;
						}
					}

					r_cg_array[i] = r_ng_array[i];		// UPDATE GRIDS
					r_ng_array[i] = robot1_grids[currentGridIndex+1];		// UPDATE GRIDS
					r_x_array[i] = -1;										// START FROM BEGINNING OF GRID

				}
				 if (i == 2){

					int currentGridIndex = 0;
					for (int x = 0; x < 8; x++){
						if (robot2_grids[x] = r_cg_array[i]){
							cout << "CURRENT GRID INDEX OF ROBOT 2 IS " << currentGridIndex << endl;
							currentGridIndex = x;
							break;
						}
					}

					r_cg_array[i] = r_ng_array[i];		// UPDATE GRIDS
					r_ng_array[i] = robot2_grids[currentGridIndex+1];		// UPDATE GRIDS
					r_x_array[i] = -1;
				}
				
				
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
	
	for (int i = 0; i <8 ; i++)
	{
		//cout << robot0_grids.at(i)<< endl;
		cout << robot0_grids[i] << " ";
	}
	cout << endl;
	for (int i = 0; i <8; i++)
	{
		//cout << robot0_grids.at(i)<< endl;
		cout << robot1_grids[i] << " ";
	}
	cout << endl;

		for (int i = 0; i <8; i++)
		{
			//cout << robot0_grids.at(i)<< endl;
			cout << robot2_grids[i] << " ";
		}
		cout << endl;

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

void ENVIRONMENT<environment_T>::prc_robot_path()
{
	for (;;){
		if (robot_start_moving_port[0].read() == 1) //Checks if robot0 has started moving
		{
			//WRITE VALUES IN FIFO
			robot0_n_path.write(1);
			wait(1, SC_MS);
			robot0_n_path.write(2);
			wait(1, SC_MS);
			robot0_n_path.write(3);
			wait(1, SC_MS);
			robot0_n_path.write(4);
			wait(1, SC_MS);
			robot0_n_path.write(14);
			wait(1, SC_MS);
			robot0_n_path.write(24);
			wait(1, SC_MS);
			robot0_n_path.write(34);
			wait(1, SC_MS);
			robot0_n_path.write(-1);
			wait(1, SC_MS);

			for (int i = 0; i < 8; i++)
			{
				//robot0_grids.push_back(robot0_n_path.read());
				robot0_grids[i] = robot0_n_path.read();

			}

		}
		if (robot_start_moving_port[1].read() == 1) //Checks if robot1 has started moving
		{
			//WRITE VALUES IN FIFO
			robot1_n_path.write(44);
			wait(1, SC_MS);
			robot1_n_path.write(34);
			wait(1, SC_MS);
			robot1_n_path.write(24);
			wait(1, SC_MS);
			robot1_n_path.write(23);
			wait(1, SC_MS);
			robot1_n_path.write(22);
			wait(1, SC_MS);
			robot1_n_path.write(32);
			wait(1, SC_MS);
			robot1_n_path.write(42);
			wait(1, SC_MS);
			robot1_n_path.write(-1);
			wait(1, SC_MS);

			for (int i = 0; i < 8; i++)
			{
				//robot1_grids.push_back(robot1_n_path.read());

				robot1_grids[i] = robot1_n_path.read();

			}
		}
		if (robot_start_moving_port[2].read() == 1) //Checks if robot2 has started moving
		{
			//WRITE VALUES IN FIFO
			robot2_n_path.write(20);
			wait(1, SC_MS);
			robot2_n_path.write(21);
			wait(1, SC_MS);
			robot2_n_path.write(22);
			wait(1, SC_MS);
			robot2_n_path.write(23);
			wait(1, SC_MS);
			robot2_n_path.write(24);
			wait(1, SC_MS);
			robot2_n_path.write(14);
			wait(1, SC_MS);
			robot2_n_path.write(4);
			wait(1, SC_MS);
			robot2_n_path.write(-1);
			wait(1, SC_MS);

			for (int i = 0; i < 8; i++)
			{
				//robot2_grids.push_back(robot2_n_path.read());
				robot2_grids[i] = robot2_n_path.read();
			}

		}

	}
}

void ENVIRONMENT<environment_T>::prc_print_environment(){


	cout << endl << "~~~~~~~~~~~~~~~~~~~~~~~ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
	cout << endl << "===========================ROBOT ARRAY===========================" << endl;
	cout << "|RI\t|CG\t|NG\t|X\t|Y\t|MOVING\t|BOUND|GRID\t|OBST\t|" << endl;
	cout << "=================================================================" << endl;
	for (int i = 0; i < *(_numRobots); i++){

		e_robot_array[i][0] = r_index_array[i];
		e_robot_array[i][1] = r_cg_array[i];
		e_robot_array[i][2] = r_ng_array[i];
		e_robot_array[i][3] = r_x_array[i];
		e_robot_array[i][4] = r_y_array[i];
		e_robot_array[i][5] = robot_start_moving_port[i].read();
		e_robot_array[i][6] = boundary_port[i].read();
		e_robot_array[i][7] = checkingBoundary[i];
		e_robot_array[i][8] = detectedObstacle[i];
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
