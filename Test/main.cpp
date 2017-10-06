#include "ENVIRONMENT.h"
#include "SERVER.h"
#include "ROBOT.h"
#define _CRT_SECURE_NO_WARNINGS

const int NUM_ROBOTS = 3;
const int NUM_OBSTACLES = 2;

int sc_main(int argc, char *argv[]){
	// DEFINE SIGNALS
	sc_clock					clk_sig;
	sc_signal<sc_uint<8> >		location_sig;
	sc_signal<bool>				serverStatus_sig[NUM_ROBOTS];
	sc_signal<bool>				robotStatus_sig[NUM_ROBOTS];
	sc_signal<bool>				environmentStatus_sig[NUM_ROBOTS];

	sc_signal<sc_uint<8> >		robotIDArray_sig[NUM_ROBOTS];
	sc_signal<sc_uint<8> >		currentPositionArray_sig[NUM_ROBOTS];
	sc_signal<sc_uint<8> >		nextPositionArray_sig[NUM_ROBOTS];

	// CREATE ROBOT 0
	typedef int robot_T;
	const robot_T robotID0 = 0;
	const robot_T speed0 = 5;
	robot_T location0 = 5;
	ROBOT<robot_T>	robot0("robot0", &robotID0, &speed0, &location0);
	robot0.clock(clk_sig);
	robot0.robotStatus_port(robotStatus_sig[0]);
	robot0.serverStatus_port(serverStatus_sig[0]);
	robot0.environmentStatus_port(environmentStatus_sig[0]);
	robot0.currentPositionArray(currentPositionArray_sig[0]);
	robot0.nextPositionArray(nextPositionArray_sig[0]);
	robot0.robotID(robotIDArray_sig[0]);

	// CREATE ROBOT 1
	typedef int robot_T;
	const robot_T robotID1 = 1;
	const robot_T speed1 = 3;
	robot_T location1 = 50;
	ROBOT<robot_T>	robot1("robot1", &robotID1, &speed1, &location1);
	robot1.clock(clk_sig);
	robot1.robotStatus_port(robotStatus_sig[1]);
	robot1.serverStatus_port(serverStatus_sig[1]);
	robot1.environmentStatus_port(environmentStatus_sig[1]);
	robot1.currentPositionArray(currentPositionArray_sig[1]);
	robot1.nextPositionArray(nextPositionArray_sig[1]);
	robot1.robotID(robotIDArray_sig[1]);

	// CREATE ROBOT 2
	typedef int robot_T;
	const robot_T robotID2 = 2;
	const robot_T speed2 = 2;
	robot_T location2 = 75;
	ROBOT<robot_T>	robot2("robot2", &robotID2, &speed2, &location2);
	robot2.clock(clk_sig);
	robot2.robotStatus_port(robotStatus_sig[2]);
	robot2.serverStatus_port(serverStatus_sig[2]);
	robot2.environmentStatus_port(environmentStatus_sig[2]);
	robot2.currentPositionArray(currentPositionArray_sig[2]);
	robot2.nextPositionArray(nextPositionArray_sig[2]);
	robot2.robotID(robotIDArray_sig[2]);

	// CREATE SERVER 
	typedef int server_T;
	const server_T server_numRobots = NUM_ROBOTS;
	SERVER<server_T>	server1("server1", &server_numRobots);
	server1.clock(clk_sig);
	
	for (int i = 0; i < NUM_ROBOTS; i++){
		server1.robotIDArray_port[i](robotIDArray_sig[i]);
		server1.currentPositionArray_port[i](currentPositionArray_sig[i]);
		server1.nextPositionArray_port[i](nextPositionArray_sig[i]);
		server1.robotStatus_port[i](robotStatus_sig[i]);
		server1.serverStatus_port[i](serverStatus_sig[i]);
	}

	// CREATE ENVIRONMENT 
	typedef int environment_T;
	const environment_T environment_numRobots = NUM_ROBOTS;
	const environment_T numObstacles = NUM_OBSTACLES;
	ENVIRONMENT<environment_T>	environment1("environment", &environment_numRobots, &numObstacles);
	environment1.clock(clk_sig);
	

	for (int i = 0; i < NUM_ROBOTS; i++){
		environment1.r_robotIDArray_port[i](robotIDArray_sig[i]);
		environment1.r_currentPositionArray_port[i](currentPositionArray_sig[i]);
		environment1.r_nextPositionArray_port[i](nextPositionArray_sig[i]);
		environment1.r_robotStatus_port[i](robotStatus_sig[i]);
		environment1.r_environmentStatus_port[i](environmentStatus_sig[i]);
	}

	sc_start(10, SC_NS);

	return 0;
}