#include "SERVER.h"
typedef int server_T;
void SERVER<server_T>::prc_server(){

	//// FOR ALL ROBOTS
	for (int thisRobot = 0; thisRobot < *(_numRobots); thisRobot++){

		// ROBOT STOPPED (BECAUSE OF BOUNDARY)
		// Server(in) <-- ROBOT(inout) <-- Environment(out)
		if (boundary_port[thisRobot].read() == 1){

			// CHECK ALL OTHER ROBOTS CURRENT GRID
			for (int otherRobot = 0; otherRobot < *(_numRobots); otherRobot++){

				// ONLY OTHER ROBOTS
				if (otherRobot != thisRobot){
					// IF THIS ROBOT's NG IS EQUAL TO ANOTHER ROBOT'S CURRENT GRID
					// TELL THIS ROBOT TO STOP B/C THERE IS ANOTHER ROBOT IN NEXT GRID

					// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
					if (r_ng_array[thisRobot] != r_cg_array[otherRobot]){
						//r_status_array[thisRobot] = 0;
						r_cg_array[thisRobot] += 1;				//Increments current grid of robot in server
						r_ng_array[thisRobot] += 1;				//Increments next grid of robot in server
						gridUpdate_port[thisRobot].write(1);	//Sends a signal for environment to update its grid in data structure


					}
					else{
						gridUpdate_port[thisRobot].write(0);
					}
				}
			}
		}
		else{
			gridUpdate_port[thisRobot].write(0);
		}
	}
}


void SERVER<server_T>::print_server(){

	cout << endl << "~~~~~~~~~~~~~Server~~~~~~~~~~~~~~" << endl;
	cout << "=================================" << endl;
	// THEN READ DATA STRUCTURE FROM LOCAL ARRAYS
	cout << "|RI\t|CG\t|NG\t|STATUS\t|" << endl;
	for (int i = 0; i < *(_numRobots); i++){
		server_array[i][0] = r_index_array[i];
		server_array[i][1] = r_cg_array[i];
		server_array[i][2] = r_ng_array[i];
		//server_array[i][3] = r_status_array[i];

		cout << "|";
		for (int j = 0; j < NUM_COLUMNS; j++){
			cout << server_array[i][j] << "\t|";
		}
		cout << endl;
	}
	cout << "=================================" << endl;

}