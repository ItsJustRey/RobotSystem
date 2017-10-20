#include "systemc.h"
#define _CRT_SECURE_NO_WARNINGS
#include <cmath>
#include <vector>
using namespace std;
const int ENVIRONMENT_MAX_ROBOTS = 5;
const int ENVIRONMENT_MAX_OBSTACLES = 5;
const int ROBOT_NUM_COLUMNS = 10;
const int OBSTACLE_NUM_COLUMNS = 5;
const int E_NUM_ROBOTS = 3;
const int E_NUM_OBSTACLES = 2;
const int GRID_WIDTH = 15;
const int GRID_HEIGHT = 15;

const int NUM_NODES = 4;
const int NODES_NUM_COLUMNS = 4;

template <class T> class ENVIRONMENT : public sc_module{
public:
	//	PORTS
	sc_in<bool> clock;

	sc_in<bool> gridUpdate_port[E_NUM_ROBOTS];							// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
	sc_out<bool>obstacle_port[E_NUM_ROBOTS];							// Server(in) <-- ROBOT(inout) <-- Environment(out)
	sc_out<bool> boundary_port[E_NUM_ROBOTS];							// Server(in) <-- ROBOT(inout) <-- Environment(out)

	sc_in<bool>	robot_start_moving_port[E_NUM_ROBOTS];					// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)



	sc_in <sc_uint<8> > r_id_port[E_NUM_ROBOTS];						// USED TO IDENTIFY EACH ROBOT

	sc_int<8>  r_index_array[E_NUM_ROBOTS];								// COLUMN FOR EACH ROBOT's INDEX
	sc_int<8>  r_cg_array[E_NUM_ROBOTS];								// COLUMN FOR EACH ROBOT's CURRENT GRID
	sc_int<8>  r_ng_array[E_NUM_ROBOTS];								// COLUMN FOR EACH ROBOT's NEXT GRID
	sc_int<8>	r_x_array[E_NUM_ROBOTS];								// COLUMN FOR EACH ROBOT's X coordinate within Grid
	sc_int<8>	r_y_array[E_NUM_ROBOTS];								// COLUMN FOR EACH ROBOT's Y coordinate within Grid
	sc_int<8>  r_status_array[E_NUM_ROBOTS];							// COLUMN FOR EACH ROBOT's STATUS
	sc_int<8>  r_speed_array[E_NUM_ROBOTS];								// COLUMN FOR EACH ROBOT's SPEED
	sc_int<8>  e_robot_array[E_NUM_ROBOTS][ROBOT_NUM_COLUMNS];			// ENVIRONMENT DATA STRUCTURE (ROBOT)

	sc_int<8>  o_index_array[E_NUM_OBSTACLES];							// COLUMN FOR EACH OBSTACLE's INDEX
	sc_int<8>  o_cg_array[E_NUM_OBSTACLES];								// COLUMN FOR EACH OBSTACLE's CURRENT GRID
	sc_int<8>  o_ng_array[E_NUM_OBSTACLES];								// COLUMN FOR EACH OBSTACLE's NEXT GRID
	sc_int<8>	o_x_array[E_NUM_OBSTACLES];								// COLUMN FOR EACH OBSTACLE's X coordinate within Grid
	sc_int<8>	o_y_array[E_NUM_OBSTACLES];								// COLUMN FOR EACH OBSTACLE's Y coordinate within Grid
	sc_int<8>  e_obstacle_array[E_NUM_OBSTACLES][OBSTACLE_NUM_COLUMNS];	// ENVIRONMENT DATA STRUCTURE (OBSTACLE)

	sc_signal<bool> checkingBoundary[E_NUM_ROBOTS];						// USED TO HOLD THE ROBOT WHILE CHECKING BOUNDARY
	sc_signal<bool> detectedObstacle[E_NUM_ROBOTS];						// USED TO HOLD THE ROBOT WHILE CHECKING OBSTACLE
	sc_signal<int> speedControl[E_NUM_ROBOTS];							// USED TO CONTROL SPEED OF EACH ROBOT....-1 = slow down       0 = normal       1 = speed up
		


	sc_in<sc_int<8> >e_cg_array_port[E_NUM_ROBOTS];
	sc_in<sc_int<8> >e_ng_array_port[E_NUM_ROBOTS];


	

	sc_in<sc_int<8> >	block0_array_port_in[6];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
	sc_in<sc_int<8> >	block1_array_port_in[6];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
	sc_in<sc_int<8> >	block2_array_port_in[6];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)



	sc_int<8> block0_array[6];
	sc_int<8> block1_array[6];
	sc_int<8> block2_array[6];



	// NODES

	//sc_int<8> nodes_index[NUM_NODES] = [0, 4, 20, 22, 24];


	sc_int<8> node4_robots_in_path[E_NUM_ROBOTS];
	sc_int<8> node20_robots_in_path[E_NUM_ROBOTS];
	sc_int<8> node22_robots_in_path[E_NUM_ROBOTS];
	sc_int<8> node24_robots_in_path[E_NUM_ROBOTS];

	sc_int<8> node4_priority[E_NUM_ROBOTS];
	sc_int<8> node20_priority[E_NUM_ROBOTS];
	sc_int<8> node22_priority[E_NUM_ROBOTS];
	sc_int<8> node24_priority[E_NUM_ROBOTS];
	//sc_int<8>  e_nodes_array[NUM_NODES][NODES_NUM_COLUMNS];	// ENVIRONMENT DATA STRUCTURE (NODES)

	sc_int<1> r0;
	sc_int<1> r1;
	sc_int<1> r2;

	void prc_environment();
	void prc_robot0_obstacle_detected();
	void prc_robot1_obstacle_detected();

	void prc_nodes();
	void prc_speed_control();
	void prc_speed_control0();
	void prc_speed_control1();
	void prc_speed_control2();
	void prc_print_environment();


	SC_HAS_PROCESS(ENVIRONMENT);
	ENVIRONMENT(sc_module_name name, const T* numRobots, const T* numObstacles, const T* r0_id, const T* r0_speed, const T* r0_grid, const T* r0_x, const T* r0_y, 
																				const T* r1_id, const T* r1_speed, const T* r1_grid, const T* r1_x, const T* r1_y, 
																				const T* r2_id, const T* r2_speed, const T* r2_grid, const T* r2_x, const T* r2_y) :
				sc_module(name), _numRobots(numRobots), _numObstacles(numObstacles), _r0_id(r0_id), _r0_speed(r0_speed), _r0_grid(r0_grid), _r0_x(r0_x), _r0_y(r0_y), 
																					_r1_id(r1_id), _r1_speed(r1_speed), _r1_grid(r1_grid), _r1_x(r1_x), _r1_y(r1_y), 
																					_r2_id(r2_id), _r2_speed(r2_speed), _r2_grid(r2_grid), _r2_x(r2_x), _r2_y(r2_y)
	{

		r_index_array[*(r0_id)] = *(r0_id);
		r_cg_array[*(r0_id)] = *(r0_grid);
		r_ng_array[*(r0_id)] = 2;
		r_x_array[*(r0_id)] = *(r0_x);
		r_y_array[*(r0_id)] = *(r0_y);
		r_status_array[*(r0_id)] = 1;
		r_speed_array[*(r0_id)] = *(r0_speed);

		r_index_array[*(r1_id)] = *(r1_id);
		r_cg_array[*(r1_id)] = *(r1_grid);
		r_ng_array[*(r1_id)] = 34;
		r_x_array[*(r1_id)] = *(r1_x);
		r_y_array[*(r1_id)] = *(r1_y);
		r_status_array[*(r1_id)] = 1;
		r_speed_array[*(r1_id)] = *(r1_speed);

		r_index_array[*(r2_id)] = *(r2_id);
		r_cg_array[*(r2_id)] = *(r2_grid);
		r_ng_array[*(r2_id)] = 21;
		r_x_array[*(r2_id)] = *(r2_x);
		r_y_array[*(r2_id)] = *(r2_y);
		r_status_array[*(r2_id)] = 1;
		r_speed_array[*(r2_id)] = *(r2_speed);



		for (int i = 0; i < *(_numRobots); i++){
			e_robot_array[i][0] = r_index_array[i];
			e_robot_array[i][1] = r_cg_array[i];
			e_robot_array[i][2] = r_ng_array[i];
			e_robot_array[i][3] = r_x_array[i];
			e_robot_array[i][4] = r_y_array[i];
			e_robot_array[i][5] = r_status_array[i];
			e_robot_array[i][6] = r_speed_array[i];
		}
		
		for (int i = 0; i < *(_numRobots); i++){
			node4_robots_in_path[i] = 0;
			node20_robots_in_path[i] = 0;
			node22_robots_in_path[i] = 0;
			node24_robots_in_path[i] = 0;
		}
			

		for (int i = 0; i < *(_numRobots); i++){
			node4_priority[i] = 0;
			node20_priority[i] = 0;
			node22_priority[i] = 0;
			node24_priority[i] = 0;
		}

		o_index_array[0] = 0;
		o_cg_array[0] = 10;
		o_ng_array[0] = 0;
		o_x_array[0] = 3;
		o_y_array[0] = 0;

		o_index_array[1] = 1;
		o_cg_array[1] = 10;
		o_ng_array[1] = 2;
		o_x_array[1] = 4;
		o_y_array[1] = 4;


		for (int i = 0; i < *(numRobots); i++){
			detectedObstacle[i] = 0;
		}
		

		cout << "CREATING ENVIRONMENT..." << "\tName: " << name << "\t# of Robots: " << *(_numRobots) << "\t# of Obstacles: " << *(_numObstacles) << endl;
		
		SC_METHOD(prc_environment);
		sensitive << clock.pos();
		dont_initialize();
		SC_METHOD(prc_robot0_obstacle_detected);
		sensitive << detectedObstacle[0].posedge_event();
		dont_initialize();
		SC_METHOD(prc_robot1_obstacle_detected);
		sensitive << detectedObstacle[1].posedge_event();
		dont_initialize();

		SC_METHOD(prc_nodes);
		sensitive << clock.pos();
		dont_initialize();

		SC_METHOD(prc_speed_control);
		//for (int i = 0; i < E_NUM_ROBOTS; i++){
		//	sensitive << speedControl[i].value_changed_event();
		//}

		sensitive << clock.pos();
		dont_initialize();

		/*SC_METHOD(prc_speed_control1);
		sensitive << slowDown[1].posedge_event();
		dont_initialize();

		SC_METHOD(prc_speed_control2);
		sensitive << slowDown[2].posedge_event();
		dont_initialize();*/


		SC_METHOD(prc_print_environment);
		sensitive << clock.pos();
		dont_initialize();



	
		

	}
	
private:
	const T* _numRobots;
	const T* _numObstacles;

	const T* _r0_id;
	const T* _r0_speed;
	const T* _r0_grid;
	const T* _r0_x;
	const T* _r0_y;

	const T* _r1_id;
	const T* _r1_speed;
	const T* _r1_grid;
	const T* _r1_x;
	const T* _r1_y;


	const T* _r2_id;
	const T* _r2_speed;
	const T* _r2_grid;
	const T* _r2_x;
	const T* _r2_y;

};