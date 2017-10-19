#include "ENVIRONMENT.h"
#include "SERVER.h"
#include "ROBOT.h"
#define _CRT_SECURE_NO_WARNINGS

const int NUM_ROBOTS = 3;
const int NUM_OBSTACLES = 2;

int sc_main(int argc, char *argv[]){

	sc_core::sc_report_handler::set_actions("/IEEE_Std_1666/deprecated",
		sc_core::SC_DO_NOTHING);

	// DEFINE SIGNALS
	sc_clock					clk_sig("clk", 1, SC_SEC);
	sc_signal<sc_uint<8> >		location_sig;

	sc_signal<sc_uint<8> >		r_id_array_sig[NUM_ROBOTS];				// ROBOT -> SERVER  
																		// ROBOT -> ENVIRONMENT

	sc_signal<bool>				r_status_array_sig[NUM_ROBOTS];			// ROBOT -> SERVER
																		// ROBOT -> ENVIRONMENT

	sc_signal<bool>				s_status_array_sig[NUM_ROBOTS];			// SERVER -> ROBOT
	
	sc_signal<bool>				e_status_array_sig[NUM_ROBOTS];			// ENVIRONMENT -> ROBOT

	sc_signal<bool>             boundary_sig[NUM_ROBOTS];

	sc_signal<bool>             gridUpdate_sig[NUM_ROBOTS];

	sc_signal<bool>				obstacle_sig[NUM_ROBOTS];


	sc_signal<bool>				s_start_robot_sig[NUM_ROBOTS];				// SIGNAL WRITTEN ONLY TO SERVER

	sc_signal<bool>				fifo_start_sig[NUM_ROBOTS];

	sc_signal<bool>				robot_start_moving_sig[NUM_ROBOTS];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)

	sc_signal<sc_int<8> >				e_cg_array_sig[NUM_ROBOTS];

	sc_signal<sc_int<8> >				e_ng_array_sig[NUM_ROBOTS];

	// CREATE ROBOT 0
	typedef int robot_T;
	const robot_T r0_id = 0;
	const robot_T r0_speed = 1;
	robot_T r0_grid = 1;
	robot_T r0_x = 0;
	robot_T r0_y = 0;
	ROBOT<robot_T>	robot0("robot0", &r0_id, &r0_speed, &r0_grid, &r0_x, &r0_y);
	robot0.clock(clk_sig);
	robot0.id_port(r_id_array_sig[r0_id]);
	robot0.boundary_port(boundary_sig[r0_id]);
	robot0.gridUpdate_port(gridUpdate_sig[r0_id]);
	robot0.obstacle_port(obstacle_sig[r0_id]);
	robot0.robot_start_moving_port(robot_start_moving_sig[r0_id]);
	robot0.e_cg_array_port(e_cg_array_sig[r0_id]);
	robot0.e_ng_array_port(e_ng_array_sig[r0_id]);
	
	// CREATE ROBOT 1
	typedef int robot_T;
	const robot_T r1_id = 1;
	const robot_T r1_speed = 1;
	robot_T r1_grid = 44;
	robot_T r1_x = 0;
	robot_T r1_y = 4;
	ROBOT<robot_T>	robot1("robot1", &r1_id, &r1_speed, &r1_grid, &r1_x, &r1_y);
	robot1.clock(clk_sig);
	robot1.id_port(r_id_array_sig[r1_id]);
	robot1.boundary_port(boundary_sig[r1_id]);
	robot1.gridUpdate_port(gridUpdate_sig[r1_id]);
	robot1.obstacle_port(obstacle_sig[r1_id]);
	robot1.robot_start_moving_port(robot_start_moving_sig[r1_id]);
	robot1.e_cg_array_port(e_cg_array_sig[r1_id]);
	robot1.e_ng_array_port(e_ng_array_sig[r1_id]);


	// CREATE ROBOT 2
	typedef int robot_T;
	const robot_T r2_id = 2;
	const robot_T r2_speed = 1;
	robot_T r2_grid = 20;
	robot_T r2_x = 0;
	robot_T r2_y = 0;
	ROBOT<robot_T>	robot2("robot2", &r2_id, &r2_speed, &r2_grid, &r2_x, &r2_y);
	robot2.clock(clk_sig);
	robot2.id_port(r_id_array_sig[r2_id]);
	robot2.boundary_port(boundary_sig[r2_id]);
	robot2.gridUpdate_port(gridUpdate_sig[r2_id]);
	robot2.obstacle_port(obstacle_sig[r2_id]);
	robot2.robot_start_moving_port(robot_start_moving_sig[r2_id]);
	robot2.e_cg_array_port(e_cg_array_sig[r2_id]);
	robot2.e_ng_array_port(e_ng_array_sig[r2_id]);




	// CREATE SERVER 
	typedef int server_T;
	const server_T server_numRobots = NUM_ROBOTS;
	SERVER<server_T>	server1("server1", &server_numRobots, &r0_id, &r0_speed, &r0_grid, &r1_id, &r1_speed, &r1_grid, &r2_id, &r2_speed, &r2_grid);
	server1.clock(clk_sig);
	
	
	for (int i = 0; i < NUM_ROBOTS; i++){
		server1.r_id_port[i](r_id_array_sig[i]);
		server1.boundary_port[i](boundary_sig[i]);
		server1.gridUpdate_port[i](gridUpdate_sig[i]);
		server1.obstacle_port[i](obstacle_sig[i]);
		server1.e_cg_array_port[i](e_cg_array_sig[i]);
		server1.e_ng_array_port[i](e_ng_array_sig[i]);

		server1.s_start_robot_port[i](s_start_robot_sig[i]);
		server1.robot_start_moving_port[i](robot_start_moving_sig[i]);
		server1.fifo_start[i](fifo_start_sig[i]);
	
	}


	// CREATE ENVIRONMENT 
	typedef int environment_T;
	const environment_T environment_numRobots = NUM_ROBOTS;
	const environment_T numObstacles = NUM_OBSTACLES;
	//ROBOT<robot_T> robots[NUM_ROBOTS];
	ENVIRONMENT<environment_T>	environment1("environment", &environment_numRobots, &numObstacles, &r0_id, &r0_speed, &r0_grid, &r0_x, &r0_y, 
																									&r1_id, &r1_speed, &r1_grid, &r1_x, &r1_y, 
																									&r2_id, &r2_speed, &r2_grid, &r2_x, &r2_y);
	environment1.clock(clk_sig);
	for (int i = 0; i < NUM_ROBOTS; i++){
		environment1.r_id_port[i](r_id_array_sig[i]);
		environment1.boundary_port[i](boundary_sig[i]);
		environment1.gridUpdate_port[i](gridUpdate_sig[i]);
		environment1.obstacle_port[i](obstacle_sig[i]);

		environment1.e_cg_array_port[i](e_cg_array_sig[i]);
		environment1.e_ng_array_port[i](e_ng_array_sig[i]);
		environment1.robot_start_moving_port[i](robot_start_moving_sig[i]);
	}




	

	// START AT 0 seconds

	fifo_start_sig[0].write(1);
	cout << " " << endl;
	cout << "ROBOT 0 HAS ARRIVED " << sc_time_stamp() << endl;

	// START AT 10 seconds
	sc_start(10, SC_SEC);
	fifo_start_sig[1].write(1);
	cout << " " << endl;
	cout << "ROBOT 1 HAS ARRIVED " << sc_time_stamp() << endl;

	// START AT 2 seconds
	sc_start(5, SC_SEC);
	fifo_start_sig[2].write(1);
	cout << " " << endl;
	cout << "ROBOT 2 HAS ARRIVED " << sc_time_stamp() << endl;

	// CONTUINUE UNTIL 20 SECONDS
	sc_start(50, SC_SEC);

	

	return 0;
}