#include "systemc.h"
#define _CRT_SECURE_NO_WARNINGS
#include <cmath>
#include <vector>
using namespace std;
const int ENVIRONMENT_MAX_ROBOTS = 5;
const int ENVIRONMENT_MAX_OBSTACLES = 5;
const int ROBOT_NUM_COLUMNS = 6;
const int OBSTACLE_NUM_COLUMNS = 5;
const int E_NUM_ROBOTS = 2;
template <class T> class ENVIRONMENT : public sc_module{
	//test
public:
	//	PORTS
	sc_in<bool> clock;
	sc_in<bool> r_status_port[E_NUM_ROBOTS];					// USED TO READ CONTROL FROM EACH ROBOT 
	sc_out<bool> e_status_port[E_NUM_ROBOTS];					// USED TO TRANSFER CONTROL TO EACH ROBOT

	sc_in <sc_uint<8> > r_id_port[E_NUM_ROBOTS];				// USED TO IDENTIFY EACH ROBOT


	sc_uint<8>  r_index_array[ENVIRONMENT_MAX_ROBOTS];								// COLUMN FOR EACH ROBOT's INDEX
	sc_uint<8>  r_cg_array[ENVIRONMENT_MAX_ROBOTS];									// COLUMN FOR EACH ROBOT's CURRENT GRID
	sc_uint<8>  r_ng_array[ENVIRONMENT_MAX_ROBOTS];									// COLUMN FOR EACH ROBOT's NEXT GRID
	sc_uint<8>	r_x_array[ENVIRONMENT_MAX_ROBOTS];									// COLUMN FOR EACH ROBOT's X coordinate within Grid
	sc_uint<8>	r_y_array[ENVIRONMENT_MAX_ROBOTS];									// COLUMN FOR EACH ROBOT's Y coordinate within Grid
	sc_uint<8>  r_status_array[ENVIRONMENT_MAX_ROBOTS];								// COLUMN FOR EACH ROBOT's STATUS
	sc_uint<8>  e_robot_array[ENVIRONMENT_MAX_ROBOTS][ROBOT_NUM_COLUMNS];			// ENVIRONMENT DATA STRUCTURE (ROBOT)

	sc_uint<8>  o_index_array[ENVIRONMENT_MAX_OBSTACLES];							// COLUMN FOR EACH OBSTACLE's INDEX
	sc_uint<8>  o_cg_array[ENVIRONMENT_MAX_OBSTACLES];								// COLUMN FOR EACH OBSTACLE's CURRENT GRID
	sc_uint<8>  o_ng_array[ENVIRONMENT_MAX_OBSTACLES];								// COLUMN FOR EACH OBSTACLE's NEXT GRID
	sc_uint<8>	o_x_array[ENVIRONMENT_MAX_OBSTACLES];								// COLUMN FOR EACH OBSTACLE's X coordinate within Grid
	sc_uint<8>	o_y_array[ENVIRONMENT_MAX_OBSTACLES];								// COLUMN FOR EACH OBSTACLE's Y coordinate within Grid
	sc_uint<8>  e_obstacle_array[ENVIRONMENT_MAX_OBSTACLES][OBSTACLE_NUM_COLUMNS];	// ENVIRONMENT DATA STRUCTURE (OBSTACLE)

	SC_HAS_PROCESS(ENVIRONMENT);
	ENVIRONMENT(sc_module_name name, const T* numRobots, const T* numObstacles, const T* r0_id, const T* r0_speed, const T* r0_grid, const T* r0_x, const T* r0_y, const T* r1_id, const T* r1_speed, const T* r1_grid, const T* r1_x, const T* r1_y) :
		sc_module(name), _numRobots(numRobots), _numObstacles(numObstacles), _r0_id(r0_id), _r0_speed(r0_speed), _r0_grid(r0_grid), _r0_x(r0_x), _r0_y(r0_y),  _r1_id(r1_id), _r1_speed(r1_speed), _r1_grid(r1_grid), _r1_x(r1_x), _r1_y(r1_y)
	{
		r_index_array[*(r0_id)] = *(r0_id);
		r_cg_array[*(r0_id)] = *(r0_grid);
		r_ng_array[*(r0_id)] = *(r0_grid) + 1;
		r_x_array[*(r0_id)] = *(r0_x);
		r_y_array[*(r0_id)] = *(r0_y);
		r_status_array[*(r0_id)] = 1;

		r_index_array[*(r1_id)] = *(r1_id);
		r_cg_array[*(r1_id)] = *(r1_grid);
		r_ng_array[*(r1_id)] = *(r1_grid) + 1;
		r_x_array[*(r1_id)] = *(r1_x);
		r_y_array[*(r1_id)] = *(r1_y);
		r_status_array[*(r1_id)] = 1;

		for (int i = 0; i < *(_numRobots); i++){
			e_robot_array[i][0] = r_index_array[i];
			e_robot_array[i][1] = r_cg_array[i];
			e_robot_array[i][2] = r_ng_array[i];
			e_robot_array[i][3] = r_x_array[i];
			e_robot_array[i][4] = r_y_array[i];
			e_robot_array[i][5] = r_status_array[i];
		}
		


		cout << "CREATING ENVIRONMENT..." << "\tName: " << name << "\t# of Robots: " << *(_numRobots) << "\t# of Obstacles: " << *(_numObstacles) << endl;
		SC_CTHREAD(run1, clock.pos());
		SC_METHOD(prc_environment);
		sensitive << clock.pos();

	}
	void prc_environment(){
		// FOR EACH ROBOT, UPDATE DATA STRUCTURES
		for (int i = 0; i < *(_numRobots); i++){
			r_index_array[i] = r_id_port[i].read();
			r_status_array[i] = r_status_port[i].read();


		}
		o_index_array[0] = 0;
		o_cg_array[0] = 2;
		o_ng_array[0] = 2;
		o_x_array[0] = 0;
		o_y_array[0] = 0;

		o_index_array[1] = 1;
		o_cg_array[1] = 3;
		o_ng_array[1] = 3;
		o_x_array[1] = 5;
		o_y_array[1] = 5;

		


		/*
			//}

			//// FOR EACH ROBOT, CHECK IF ROBOT IS CLOSE TO BOUNDARY/OTHER ROBOTS
			//for (int i = 0; i < *(_numRobots); i++){

			//	//int thisRobotCurrentGrid = r_currentGridArray[i];

			//	if (r_statusArray[i] == 1){
			//		r_environmentStatus_port[i].write(1);
			//	}
			//	else{
			//		r_environmentStatus_port[i].write(0);
			//	}
			//}

			////FOR EACH ROBOT, COMPARE WITH OBSTACLES
			//for (int i = 0; i < *(_numRobots); i++){
			//	cout << endl << endl;
			//	int numObstaclesFront = 0;
			//	int numObstaclesBehind = 0;
			//	int thisRobotSpeed = r_nextGridArray[i] - r_currentGridArray[i];

			//	vector<int> obstaclesInFront;


			//	for (int j = 0; j < *(_numObstacles); j++){
			//		if (r_currentGridArray[i] < o_currentGridArray[j]){
			//			cout << "ROBOT " << i << " saw obstacle " << j << " FRONT of it." << endl;
			//			numObstaclesFront++;
			//			obstaclesInFront.push_back(j);
			//		}
			//		else{
			//			cout << "ROBOT " << i << " saw obstacle " << j << " BEHIND of it." << endl;
			//			numObstaclesBehind++;
			//		}
			//	}


			//	if (obstaclesInFront.size() > 0){
			//		cout << "ROBOT " << i << " Has obstacles: ";
			//		for (int k = 0; k < numObstaclesFront; k++){
			//			cout << obstaclesInFront.at(k) << ".";

			//		}
			//		cout << " in front of it" <<endl;

			//		for (int k = 0; k < numObstaclesFront; k++){
			//			int indexObstacleInFront = obstaclesInFront.at(k);

			//					// HAVE TO COMPARE THE OBJECT IN FRONT OF IT ONLY
			//					cout << " This Robot " << i << " next grid is " << r_nextGridArray[i] << endl;

			//					if (r_nextGridArray[i] >= o_currentGridArray[indexObstacleInFront] ){
			//						cout << " WRITING A 0 TO ENBIRONMENT STATUS" << endl;
			//						r_environmentStatus_port[i].write(0);
			//						break;
			//					}
			//					else{
			//						r_environmentStatus_port[i].write(1);
			//					}
			//		}
			//	}
			//	else{
			//		cout << "ROBOT " << i << " Has NO obstacles in front of it";
			//	}

			//cout << "ROBOT " << i << " Has " << obstaclesBehind << " obstacles BEHIND it." << endl;
			//r_environmentStatus_port[i].write(0);
			//if (obstaclesInFront.size() > 0){
			//	for (int f = 0; f < obstaclesInFront.size(); f++){

			//		int indexObstacleInFront = obstaclesInFront.at(f);

			//		// HAVE TO COMPARE THE OBJECT IN FRONT OF IT ONLY
			//		if (r_nextGridArray[i] > o_currentGridArray[indexObstacleInFront]){
			//			r_environmentStatus_port[i].write(0);
			//		}
			//		else{
			//			r_environmentStatus_port[i].write(1);
			//		}

			//	}
			//}
			//else{
			//	r_environmentStatus_port[i].write(0);

			//}




			for (int j = 0; j < *(_numObstacles); j++){
			if (numObstaclesFront > 0){
			//if ((r_currentGridArray[i] < o_currentGridArray[j]) && (r_nextGridArray[i] >= o_currentGridArray[j])){

			// HAVE TO COMPARE THE OBJECT IN FRONT OF IT ONLY
			if (r_nextGridArray[i] > o_currentGridArray[j]){
			r_environmentStatus_port[i].write(0);
			}
			else{
			r_environmentStatus_port[i].write(1);
			}
			}
			else{
			r_environmentStatus_port[i].write(1);
			}
			}


			++;
			}
			else if (obstaclesInFront = 0){
			r_environmentStatus_port[i].write(1);
			}


			// IF THIS ROBOT IS WITHIN THIS ROBOT's NEXT GRID of an obstacle, tell the robot to stop
			int distanceToObstacle = o_currentGridArray[j] - r_nextGridArray[i];

			if (distanceToObstacle < 0){
			distanceToObstacle = distanceToObstacle * (-1);
			}


			//// THIS OBJECT IS BEHIND THE ROBOT, IGNORE
			//if (o_currentGridArray[j] <= r_currentGridArray[i]){
			//	break;
			//}

			//if (r_nextGridArray[i] <= o_currentGridArray[j]){
			//	r_environmentStatus_port[i].write(0);
			//}
			//else{

			//}
			if (r_nextGridArray[i] - o_currentGridArray[j] <=  thisRobotSpeed){

			//if (n[j] - r_currentGridArray[i] < (r_nextGridArray[i] - r_currentGridArray[i])){
			//if (r_nextGridArray[i] >= o_currentGridArrayy[j]){
			//if (r_nextGridArray[i] <= (o_currentGridArray[j] + thisRobotSpeed) ){




			if (o_currentGridArray[j] <= )
			*/

		cout << endl << "~~~~~~~~~~~~~~~~~~~~~~~ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		cout << endl << "===================ROBOT ARRAY===================" << endl;
		cout << "|RI\t|CG\t|NG\t|X\t|Y\tSTATUS\t|" << endl;
		cout << "=================================================" << endl;
		for (int i = 0; i < *(_numRobots); i++){

			e_robot_array[i][0] = r_index_array[i];
			e_robot_array[i][1] = r_cg_array[i];
			e_robot_array[i][2] = r_ng_array[i];
			e_robot_array[i][3] = r_x_array[i];
			e_robot_array[i][4] = r_y_array[i];
			e_robot_array[i][5] = r_status_array[i];

			cout << "|";
			for (int j = 0; j < ROBOT_NUM_COLUMNS; j++){
				cout << e_robot_array[i][j] << "\t|";
			}
			cout << endl;
		}
		cout << "=================================================" << endl;

		cout << endl << "==============OBSTACLE ARRAY=============" << endl;
		cout << "|OI\t|CG\t|NG\t|X\t|Y\t|" << endl;
		cout << "=========================================" << endl;
		for (int i = 0; i < NUM_OBSTACLES; i++){

			e_obstacle_array[i][0] = o_index_array[i];
			e_obstacle_array[i][1] = o_cg_array[i];
			e_obstacle_array[i][2] = o_ng_array[i];
			e_obstacle_array[i][3] = o_x_array[i];
			e_obstacle_array[i][4] = o_y_array[i];
			cout << "|";
			for (int j = 0; j < OBSTACLE_NUM_COLUMNS; j++){
				cout << e_obstacle_array[i][j] << "\t|";
			}
			cout << endl;
		}
		cout << "=========================================" << endl;
	}
	void run1(){
		wait();
		cout << endl << "~~~~~~~~~~~~~~~~~~~~1st CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~ " << endl;
		wait();


		cout << endl << "~~~~~~~~~~~~~~~~~~~~2nd CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~3rd CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~4th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~5th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~6th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~7th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~8th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~9th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~10th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();


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
};