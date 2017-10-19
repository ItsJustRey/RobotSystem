#include "ROBOT.h"

typedef int robot_T;
void ROBOT<robot_T> ::prc_robot(){

	if (robot_start_moving_port.read() == 1){

		// UPDATE ROBOT PATH
		cout << "ROBOT " << *(_id) << " PATH: " ;
		for (int j = 0; j < 8; j++)
		{

			block_array[j] = block_array_port_inout[j].read();
			block_array_port_inout[j].write(block_array[j]);
			cout << "(" << block_array[j] << ") --> ";
		}

		// CHECK STATUSES

		// Server(out) -- >ROBOT(inout) --> ENVIRONMENT(in)
		if (gridUpdate_port.read() == 1)
		{
			gridUpdate_port.write(1);


		}
		else{
			gridUpdate_port.write(0);
		}

		// OBSTACLE
		// Server(in) <-- ROBOT(inout) <-- Environment(out)
		if (obstacle_port.read() == 1){
			obstacle_port.write(1);
		}
		else{
			obstacle_port.write(0);
		}

		// BOUNDARY
		// Server(in) <-- ROBOT(inout) <-- Environment(out)
		if (boundary_port.read() == 1){
			boundary_port.write(1);
		}
		else{
			boundary_port.write(0);
		}

		e_cg_array_port.write(e_cg_array_port.read());
		e_ng_array_port.write(e_ng_array_port.read());
	}
}

void ROBOT<robot_T> ::print_robot(){

	cout << endl;
	cout << "~~~~~~~~~~~~~ROBOT " << *(_id) << "~~~~~~~~~~~~~" << endl;
	cout << "=================================" << endl;
	cout << "| Moving:    " << robot_start_moving_port.read() << "\t\t\t|" << endl;
	cout << "| Grid:      " << gridUpdate_port.read() << "\t\t\t|" << endl;
	cout << "| Boundary:  " << boundary_port.read() << "\t\t\t|" << endl;

	//cout << "| Env Status:       " << e_status_port.read() << "\t\t|" << endl;
	cout << "=================================" << endl;


}




