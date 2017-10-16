#include "ROBOT.h"

typedef int robot_T;
void ROBOT<robot_T> ::prc_robot(){


	// CHECK STATUSES

	// Server(out) -- >ROBOT(inout) --> ENVIRONMENT(in)
	if (gridUpdate_port.read() == 1)
	{
		gridUpdate_port.write(1);


		// Server(in) <-- ROBOT(inout) <-- Environment(out)
		if (obstacle_port.read() == 1){
			obstacle_port.write(1);
		}
		else{
			obstacle_port.write(0);
		}



		// Server(in) <-- ROBOT(inout) <-- Environment(out)
		if (boundary_port.read() == 1){
			boundary_port.write(1);
		}
		else{
			boundary_port.write(0);
		}
	}
	else{
		gridUpdate_port.write(0);
		boundary_port.write(0);
		obstacle_port.write(0);
	}
	/*else if (gridUpdate_port.read() == 0) {
		gridUpdate_port.write(0);
	}
	else if (boundary_port.read() == 1){
		boundary_port.write(1);
	}
	else if (boundary_port.read() == 0){
		boundary_port.write(0);
	}*/


		// Server(in) <-- ROBOT(inout) <-- Environment(out)
	/*if (boundary_port.read() == 1)
	{
		boundary_port.write(1);
	}
	else{
		boundary_port.write(0);
	}
*/

	// ENVIRONMENT TOLD ROBOT TO STOP
	// Server(out) -- >ROBOT(inout) --> ENVIRONMENT(in)
	/*if (e_status_port.read() == 0){
		r_status_port.write(0);		

	}
	else{

	}*/

	// ENVIRONMENT AND SERVER LETS ROBOT MOVE
	//else if (e_status_port.read() == 1 && s_status_port.read() == 1){
	//else{
	//	r_status_port.write(1);

	//}

	
	cout << endl;
	cout << "~~~~~~~~~~~~~ROBOT " << *(_id) << "~~~~~~~~~~~~~" << endl;
	cout << "=================================" << endl;
	cout << "| Grid:      " << gridUpdate_port.read() << "\t\t|" << endl;
	cout << "| Boundary:  " << boundary_port.read() << "\t\t|" << endl;
	//cout << "| Env Status:       " << e_status_port.read() << "\t\t|" << endl;
	cout << "=================================" << endl;
}




