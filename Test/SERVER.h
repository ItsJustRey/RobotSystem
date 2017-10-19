#include "systemc.h"
#include <vector>
#include <iostream>
using namespace std;
#define _CRT_SECURE_NO_WARNINGS

const int SERVER_MAX_ROBOTS = 5;
const int NUM_COLUMNS = 6;
const int S_NUM_ROBOTS = 3;

template <class T> class SERVER : public sc_module {

public:

	sc_in<bool> clock;
	

	sc_in <sc_uint<8> > r_id_port[S_NUM_ROBOTS];			// USED TO IDENTIFY EACH ROBOT
	sc_out<bool>	gridUpdate_port[S_NUM_ROBOTS];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
	sc_in<bool>		obstacle_port[S_NUM_ROBOTS];			// Server(in) <-- ROBOT(inout) <-- Environment(out)
	sc_in<bool>		boundary_port[S_NUM_ROBOTS];			// Server(in) <-- ROBOT(inout) <-- Environment(out)

	sc_out<bool>	robot_start_moving_port[S_NUM_ROBOTS];		// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)

	
	sc_int<8>  r_index_array[S_NUM_ROBOTS];					// COLUMN FOR EACH ROBOT INDEX
	sc_int<8>  r_cg_array[S_NUM_ROBOTS];					// COLUMN FOR EACH ROBOT's CURRENT GRID
	sc_int<8>  r_ng_array[S_NUM_ROBOTS];					// COLUMN FOR EACH ROBOT's NEXT GRID
	sc_int<8>  r_status_array[S_NUM_ROBOTS];				// COLUMN FOR EACH ROBOT's STATUS

	sc_int<8>  server_array[S_NUM_ROBOTS][NUM_COLUMNS];	// SERVER DATA STRUCTURE 
	
	
	sc_in<bool>	s_start_robot_port[S_NUM_ROBOTS];	// WRITE TO THIS SIGNAL (CONNECTED TO SERVER from MAIN)

	sc_out<sc_int<8> >e_cg_array_port[S_NUM_ROBOTS];
	sc_out<sc_int<8> >e_ng_array_port[S_NUM_ROBOTS];
	sc_inout<bool>fifo_start[S_NUM_ROBOTS];

	sc_int<8> block0_array[8];
	sc_int<8> block1_array[8];
	sc_int<8> block2_array[8];
	
	sc_int<8>array0_count;
	sc_int<8>array1_count;
	sc_int<8>array2_count;
	
	//vector<int> block_array;

	sc_out<sc_int<8> >	block0_array_port_out[8];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
	sc_out<sc_int<8> >	block1_array_port_out[8];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
	sc_out<sc_int<8> >	block2_array_port_out[8];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)


	sc_signal<bool> checkingBoundary[S_NUM_ROBOTS];	// USED TO HOLD THE ROBOT WHILE CHECKING BOUNDARY

	void prc_server();
	void prc_robot0_start();
	void prc_robot1_start();
	void prc_robot2_start();
	void prc_robot_path();
	void print_server();

	SC_HAS_PROCESS(SERVER);

	sc_fifo <int> robot0_n_path;
	sc_fifo <int> robot1_n_path;
	sc_fifo <int> robot2_n_path;


	SERVER(sc_module_name name, const T* numRobots, const T* r0_id, const T* r0_speed, const T* r0_grid,
													const T* r1_id, const T* r1_speed, const T* r1_grid, 
													const T* r2_id, const T* r2_speed, const T* r2_grid) :
			sc_module(name), _numRobots(numRobots), _r0_id(r0_id), _r0_speed(r0_speed), _r0_grid(r0_grid),
													_r1_id(r1_id), _r1_speed(r1_speed), _r1_grid(r1_grid), 
													_r2_id(r2_id), _r2_speed(r2_speed), _r2_grid(r2_grid)
	{
		r_index_array[*(r0_id)] = *(r0_id);
		r_cg_array[*(r0_id)] = *(r0_grid);
		r_ng_array[*(r0_id)] = 2;
		r_status_array[*(r0_id)] = 1;

		r_index_array[*(r1_id)] = *(r1_id);
		r_cg_array[*(r1_id)] = *(r1_grid);
		r_ng_array[*(r1_id)] = 34;
		r_status_array[*(r1_id)] = 1;


		r_index_array[*(r2_id)] = *(r2_id);
		r_cg_array[*(r2_id)] = *(r2_grid);
		r_ng_array[*(r2_id)] = 21;
		r_status_array[*(r2_id)] = 1;


		for (int i = 0; i < *(_numRobots); i++){
			server_array[i][0] = r_index_array[i];
			server_array[i][1] = r_cg_array[i];
			server_array[i][2] = r_ng_array[i];
			server_array[i][3] = r_status_array[i];
		}

		sc_fifo <int> robot0_n_path(8);
		sc_fifo <int> robot1_n_path(8);
		sc_fifo <int> robot2_n_path(8);
	

		cout << "CREATING SERVER..." << "\tName: " << name << "\t# of Robots: " << *(_numRobots) << endl;

		/*SC_CTHREAD(prc_robot_path,fifo_start.pos());*/
		

		SC_METHOD(prc_server);
		sensitive << clock.pos();
		dont_initialize();
		
		SC_METHOD(prc_robot0_start);
		sensitive << fifo_start[0].pos();
		

		SC_METHOD(prc_robot1_start);
		sensitive << fifo_start[1].pos();

		SC_METHOD(prc_robot2_start);
		sensitive << fifo_start[2].pos();

		SC_METHOD(print_server);
		sensitive << clock.pos();
		dont_initialize();
	}
	
	

private:
	const T* _numRobots;

	const T* _r0_id;
	const T* _r0_speed;
	const T* _r0_grid;

	const T* _r1_id;
	const T* _r1_speed;
	const T* _r1_grid;

	const T* _r2_id;
	const T* _r2_speed;
	const T* _r2_grid;

};
