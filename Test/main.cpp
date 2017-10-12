#include "ENVIRONMENT.h"
#include "SERVER.h"
#include "ROBOT.h"
#define _CRT_SECURE_NO_WARNINGS

const int NUM_ROBOTS = 2;
const int NUM_OBSTACLES = 2;

int sc_main(int argc, char *argv[]){

	// DEFINE SIGNALS
	sc_clock					clk_sig;
	sc_signal<sc_uint<8> >		location_sig;

	sc_signal<sc_uint<8> >		r_id_array_sig[NUM_ROBOTS];				// ROBOT -> SERVER  
																		// ROBOT -> ENVIRONMENT

	sc_signal<bool>				r_status_array_sig[NUM_ROBOTS];			// ROBOT -> SERVER
																		// ROBOT -> ENVIRONMENT

	sc_signal<bool>				s_status_array_sig[NUM_ROBOTS];			// SERVER -> ROBOT
	
	sc_signal<bool>				e_status_array_sig[NUM_ROBOTS];			// ENVIRONMENT -> ROBOT

	sc_signal<bool>             boundary_sig[NUM_ROBOTS];

	sc_signal<bool>             gridUpdate_sig[NUM_ROBOTS];

	// CREATE ROBOT 0
	typedef int robot_T;
	const robot_T r0_id = 0;
	const robot_T r0_speed = 1;
	robot_T r0_grid = 0;
	robot_T r0_x = 3;
	robot_T r0_y = 0;
	ROBOT<robot_T>	robot0("robot0", &r0_id, &r0_speed, &r0_grid, &r0_x, &r0_y);
	robot0.clock(clk_sig);
	robot0.id_port(r_id_array_sig[r0_id]);
	robot0.s_status_port(s_status_array_sig[r0_id]);
	robot0.r_status_port(r_status_array_sig[r0_id]);
	robot0.e_status_port(e_status_array_sig[r0_id]);
	robot0.boundary_port(boundary_sig[r0_id]);
	robot0.gridUpdate_port(gridUpdate_sig[r0_id]);
	
	// CREATE ROBOT 1
	typedef int robot_T;
	const robot_T r1_id = 1;
	const robot_T r1_speed = 2;
	robot_T r1_grid = 1;
	robot_T r1_x = 2;
	robot_T r1_y = 3;
	ROBOT<robot_T>	robot1("robot1", &r1_id, &r1_speed, &r1_grid, &r1_x, &r1_y);
	robot1.clock(clk_sig);
	robot1.id_port(r_id_array_sig[r1_id]);
	robot1.s_status_port(s_status_array_sig[r1_id]);
	robot1.r_status_port(r_status_array_sig[r1_id]);
	robot1.e_status_port(e_status_array_sig[r1_id]);
	robot1.boundary_port(boundary_sig[r1_id]);
	robot1.gridUpdate_port(gridUpdate_sig[r1_id]);

	// CREATE SERVER 
	typedef int server_T;
	const server_T server_numRobots = NUM_ROBOTS;
	SERVER<server_T>	server1("server1", &server_numRobots, &r0_id, &r0_speed, &r0_grid, &r1_id, &r1_speed, &r1_grid);
	server1.clock(clk_sig);
	
	for (int i = 0; i < NUM_ROBOTS; i++){
		server1.r_id_port[i](r_id_array_sig[i]);
		server1.r_status_port[i](r_status_array_sig[i]);
		server1.s_status_port[i](s_status_array_sig[i]);
		server1.boundary_port[i](boundary_sig[i]);
		server1.gridUpdate_port[i](gridUpdate_sig[i]);
	}


	// CREATE ENVIRONMENT 
	typedef int environment_T;
	const environment_T environment_numRobots = NUM_ROBOTS;
	const environment_T numObstacles = NUM_OBSTACLES;
	//ROBOT<robot_T> robots[NUM_ROBOTS];
	ENVIRONMENT<environment_T>	environment1("environment", &environment_numRobots, &numObstacles, &r0_id, &r0_speed, &r0_grid, &r0_x, &r0_y, &r1_id, &r1_speed, &r1_grid, &r1_x, &r1_y);
	environment1.clock(clk_sig);
	for (int i = 0; i < NUM_ROBOTS; i++){
		environment1.r_id_port[i](r_id_array_sig[i]);
		environment1.r_status_port[i](r_status_array_sig[i]);
		environment1.e_status_port[i](e_status_array_sig[i]);
		environment1.boundary_port[i](boundary_sig[i]);
		environment1.gridUpdate_port[i](gridUpdate_sig[i]);
	}

	sc_start(20, SC_NS);

	return 0;
}