#include "SERVER.h"
typedef int server_T;
void SERVER<server_T>::prc_server(){



	//	//// FOR ALL ROBOTS
	for (int thisRobot = 0; thisRobot < *(_numRobots); thisRobot++){

		if (boundary_port[thisRobot].read() == 1){
			//count = 0;
			checkingBoundary[thisRobot] = true;
		}

		// ROBOT STOPPED (BECAUSE OF BOUNDARY)
		// Server(in) <-- ROBOT(inout) <-- Environment(out)
		if (boundary_port[thisRobot].read() == 1 && checkingBoundary[thisRobot] == true){

			// CHECK ALL OTHER ROBOTS CURRENT GRID
			for (int otherRobot = 0; otherRobot < *(_numRobots); otherRobot++){

				// ONLY OTHER ROBOTS
				if (otherRobot != thisRobot){
					// IF THIS ROBOT's NG IS EQUAL TO ANOTHER ROBOT'S CURRENT GRID
					// TELL THIS ROBOT TO STOP B/C THERE IS ANOTHER ROBOT IN NEXT GRID

					// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
					if (r_ng_array[thisRobot] != r_cg_array[otherRobot]){
						gridUpdate_port[thisRobot].write(1);	//Sends a signal for environment to update its grid in data structure
					}
					else{
						gridUpdate_port[thisRobot].write(0);

					}
				}
			}
			checkingBoundary[thisRobot] = false;


		}
		else{
			gridUpdate_port[thisRobot].write(0);
		}


		if (gridUpdate_port[thisRobot].read() == 1){
			cout << "INCREMENTING CG NG OF ROBOT " << thisRobot << endl;
///ROBOT_0~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			
			if (thisRobot == 0 && r_ng_array[thisRobot] != r_cg_array[thisRobot]) //Checks if index is equal to Robot0 and that it hasnt reached end of FIFO
			{

				r_cg_array[thisRobot] = r_ng_array[thisRobot];	//Sets current grid to next grid 

				for (int i = 0; i < 7; i++) //Goes through FIFO list to find the next path
				{

					if (block0_array[i] == r_ng_array[thisRobot] && block0_array[i + 1] > 0) //Condition to set the next grid
					{
						r_ng_array[thisRobot] = block0_array[i + 1];	//Sets next grid to the next value in FIFO
						break; //break out of loop if next grid is found
					}
				}
				e_ng_array_port[thisRobot].write(r_ng_array[thisRobot]);
				e_cg_array_port[thisRobot].write(r_cg_array[thisRobot]);
			}

			else if  (thisRobot == 0 && r_ng_array[thisRobot] == r_cg_array[thisRobot]) //Checks if the end of FIFO has been reached
			{
				robot_start_moving_port[0].write(0);			//Stop the robot
				e_ng_array_port[thisRobot].write(r_ng_array[thisRobot]);
				e_cg_array_port[thisRobot].write(r_cg_array[thisRobot]);
			}
///ROBOT_1~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			if (thisRobot == 1 && r_ng_array[thisRobot] != r_cg_array[thisRobot]) //Checks if index is equal to Robot0 and that it hasnt reached end of FIFO
			{

				r_cg_array[thisRobot] = r_ng_array[thisRobot];	//Sets current grid to next grid 

				for (int i = 0; i < 7; i++) //Goes through FIFO list to find the next path
				{

					if (block1_array[i] == r_ng_array[thisRobot] && block1_array[i + 1] > 0) //Condition to set the next grid
					{
						r_ng_array[thisRobot] = block1_array[i + 1];	//Sets next grid to the next value in FIFO
						break; //break out of loop if next grid is found
					}
				}
				e_ng_array_port[thisRobot].write(r_ng_array[thisRobot]);
				e_cg_array_port[thisRobot].write(r_cg_array[thisRobot]);
			}

			else if (thisRobot == 1 && r_ng_array[thisRobot] == r_cg_array[thisRobot]) //Checks if the end of FIFO has been reached
			{
				robot_start_moving_port[1].write(0);			//Stop the robot
				e_ng_array_port[thisRobot].write(r_ng_array[thisRobot]);
				e_cg_array_port[thisRobot].write(r_cg_array[thisRobot]);
			}
///ROBOT_2~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			if (thisRobot == 2 && r_ng_array[thisRobot] != r_cg_array[thisRobot]) //Checks if index is equal to Robot0 and that it hasnt reached end of FIFO
			{

				r_cg_array[thisRobot] = r_ng_array[thisRobot];	//Sets current grid to next grid 

				for (int i = 0; i < 7; i++) //Goes through FIFO list to find the next path
				{

					if (block2_array[i] == r_ng_array[thisRobot] && block2_array[i + 1] > 0) //Condition to set the next grid
					{
						r_ng_array[thisRobot] = block2_array[i + 1];	//Sets next grid to the next value in FIFO
						break; //break out of loop if next grid is found
					}
				}
				e_ng_array_port[thisRobot].write(r_ng_array[thisRobot]);
				e_cg_array_port[thisRobot].write(r_cg_array[thisRobot]);
			}

			else if (thisRobot == 2 && r_ng_array[thisRobot] == r_cg_array[thisRobot]) //Checks if the end of FIFO has been reached
			{
				robot_start_moving_port[2].write(0);			//Stop the robot
				e_ng_array_port[thisRobot].write(r_ng_array[thisRobot]);
				e_cg_array_port[thisRobot].write(r_cg_array[thisRobot]);
			}
		}
	}
}

void SERVER<server_T>::prc_robot0_start(){

	if (fifo_start[0].read() == 1) //Checks if robot0 has started moving
	{
		e_ng_array_port[0].write(r_ng_array[0]);
		e_cg_array_port[0].write(r_cg_array[0]);
		//WRITE VALUES IN FIFO
		robot0_n_path.write(2);
		next_trigger(1.0, SC_NS);
		robot0_n_path.write(3);
		next_trigger(1.0, SC_NS);
		robot0_n_path.write(4);
		next_trigger(1.0, SC_NS);
		robot0_n_path.write(14);
		next_trigger(1.0, SC_NS);
		robot0_n_path.write(24);
		next_trigger(1.0, SC_NS);
		robot0_n_path.write(34);
		next_trigger(1.0, SC_NS);
		robot0_n_path.write(-1);
		next_trigger(1.0, SC_NS);

		for (int i = 0; i < 7; i++)
		{
			int val;
			robot0_n_path.nb_read(val);
			block0_array[i] = val;
			array0_count++;

		}
		if (array0_count == 6)
		{
			fifo_start[0].write(0);
			robot_start_moving_port[0].write(1);
		}
		
	}

}
void SERVER<server_T>::prc_robot1_start(){
	if (fifo_start[1].read() == 1) //Checks if robot1 has started moving
	{
		e_ng_array_port[1].write(r_ng_array[1]);
		e_cg_array_port[1].write(r_cg_array[1]);
		//WRITE VALUES IN FIFO
		robot1_n_path.write(34);
		next_trigger(1.0, SC_MS);
		robot1_n_path.write(24);
		next_trigger(1.0, SC_MS);
		robot1_n_path.write(23);
		next_trigger(1.0, SC_MS);
		robot1_n_path.write(22);
		next_trigger(1.0, SC_MS);
		robot1_n_path.write(32);
		next_trigger(1.0, SC_MS);
		robot1_n_path.write(42);
		next_trigger(1.0, SC_MS);
		robot1_n_path.write(-1);
		next_trigger(1.0, SC_MS);

		for (int i = 0; i < 7; i++)
		{
			int val;
			robot1_n_path.nb_read(val);
			block1_array[i] = val;
			array1_count++;

		}
		if (array1_count == 6)
		{
			fifo_start[1].write(0);
			robot_start_moving_port[1].write(1);
		}

	}
}
void SERVER<server_T>::prc_robot2_start(){
	if (fifo_start[2].read() == 1) //Checks if robot2 has started moving
	{
		e_ng_array_port[2].write(r_ng_array[2]);
		e_cg_array_port[2].write(r_cg_array[2]);
		//WRITE VALUES IN FIFO
		robot2_n_path.write(21);
		next_trigger(1.0, SC_MS);
		robot2_n_path.write(22);
		next_trigger(1.0, SC_MS);
		robot2_n_path.write(23);
		next_trigger(1.0, SC_MS);
		robot2_n_path.write(24);
		next_trigger(1.0, SC_MS);
		robot2_n_path.write(14);
		next_trigger(1.0, SC_MS);
		robot2_n_path.write(4);
		next_trigger(1.0, SC_MS);
		robot2_n_path.write(-1);
		next_trigger(1.0, SC_MS);

		for (int i = 0; i < 7; i++)
		{
			int val;
			robot2_n_path.nb_read(val);
			block2_array[i] = val;
			array2_count++;

		}
		if (array2_count == 6)
		{
			fifo_start[2].write(0);
			robot_start_moving_port[2].write(1);
		}

	}
}
	


//void SERVER<server_T>::prc_robot_path(){
//		if (fifo_start.read() == 1) //Checks if robot0 has started moving
//		{
//			//WRITE VALUES IN FIFO
//			robot0_n_path.write(2);
//			next_trigger(1.0, SC_MS);
//			robot0_n_path.write(3);
//			next_trigger(1.0, SC_MS);
//			robot0_n_path.write(4);
//			next_trigger(1.0, SC_MS);
//			robot0_n_path.write(14);
//			next_trigger(1.0, SC_MS);
//			robot0_n_path.write(24);
//			next_trigger(1.0, SC_MS);
//			robot0_n_path.write(34);
//			next_trigger(1.0, SC_MS);
//			robot0_n_path.write(-1);
//			next_trigger(1.0, SC_MS);
//
//			for (int i = 0; i < 7; i++)
//			{
//				int val;
//				robot0_n_path.nb_read(val);
//			    block_array[i] = val;
//				array_count++;
//				
//			}
//			if (array_count == 6)
//			{
//				fifo_start.write(0);
//				cout << "ROBOT 0 HAS ARRIVED " << sc_time_stamp() << endl;
//				robot_start_moving_port[0].write(1);
//			}
//			
//		}
		//if (robot_start_moving_port[1].read() == 1) //Checks if robot1 has started moving
		//{
		//	//WRITE VALUES IN FIFO
		//	robot1_n_path.write(34);
		//
		//	robot1_n_path.write(24);

		//	robot1_n_path.write(23);
		//	
		//	robot1_n_path.write(22);
	
		//	robot1_n_path.write(32);
		//
		//	robot1_n_path.write(42);
		//	
		//	robot1_n_path.write(-1);
		//	
		//}
		//if (robot_start_moving_port[2].read() == 1) //Checks if robot2 has started moving
		//{
		//	//WRITE VALUES IN FIFO
		//	robot2_n_path.write(21);
		//	
		//	robot2_n_path.write(22);
		//	
		//	robot2_n_path.write(23);
	
		//	robot2_n_path.write(24);
		//	
		//	robot2_n_path.write(14);
		//	
		//	robot2_n_path.write(4);
		//	
		//	robot2_n_path.write(-1);
		//	
		//}
		

	/*}*/


void SERVER<server_T>::print_server(){

	cout << endl << "~~~~~~~~~~~~~Server~~~~~~~~~~~~~~" << endl;
	cout << "=================================" << endl;
	// THEN READ DATA STRUCTURE FROM LOCAL ARRAYS
	cout << "|RI\t|CG\t|NG\t|MOVING\t|BOUND\t|GRID\t" << endl;
	for (int i = 0; i < *(_numRobots); i++){
		server_array[i][0] = r_index_array[i];
		server_array[i][1] = r_cg_array[i];
		server_array[i][2] = r_ng_array[i];
		server_array[i][3] = robot_start_moving_port[i].read();
		server_array[i][4] = boundary_port[i].read();
		server_array[i][5] = gridUpdate_port[i].read();
		cout << "|";
		for (int j = 0; j < NUM_COLUMNS; j++){
			cout << server_array[i][j] << "\t|";
		}
		cout << endl;
	}
	cout << "=================================" << endl;

}