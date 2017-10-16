#include "SERVER.h"
typedef int server_T;
void SERVER<server_T>::prc_server(){

	
	
	//// FOR ALL ROBOTS
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
			r_cg_array[thisRobot] += 1;				//Increments current grid of robot in server
			r_ng_array[thisRobot] += 1;				//Increments next grid of robot in server
		}
		

		
	}
}


void SERVER<server_T>::print_server(){

	cout << endl << "~~~~~~~~~~~~~Server~~~~~~~~~~~~~~" << endl;
	cout << "=================================" << endl;
	// THEN READ DATA STRUCTURE FROM LOCAL ARRAYS
	cout << "|RI\t|CG\t|NG\t|BOUND\t|GRID\t" << endl;
	for (int i = 0; i < *(_numRobots); i++){
		server_array[i][0] = r_index_array[i];
		server_array[i][1] = r_cg_array[i];
		server_array[i][2] = r_ng_array[i];
		server_array[i][3] = boundary_port[i].read();
		server_array[i][4] = gridUpdate_port[i].read();
		cout << "|";
		for (int j = 0; j < NUM_COLUMNS; j++){
			cout << server_array[i][j] << "\t|";
		}
		cout << endl;
	}
	cout << "=================================" << endl;

}