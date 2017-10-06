#include "systemc.h"
#define _CRT_SECURE_NO_WARNINGS
const int ENVIRONMENT_MAX_ROBOTS = 5;
const int ENVIRONMENT_MAX_OBSTACLES = 5;
const int ROBOT_NUM_COLUMNS = 4;
const int OBSTACLE_NUM_COLUMNS = 3;
#include <cmath>
template <class T> class ENVIRONMENT : public sc_module{
	//suckaniggadickforaniphone6...
public:
	//	PORTS
	sc_in<bool> clock;
	sc_in<bool> r_robotStatus_port[3];
	sc_inout<bool> r_environmentStatus_port[3];
	

	//sc_in <sc_vector<ROBOT<robot_T> > > robots;
	sc_inout <sc_uint<8> > r_robotIDArray_port[3];
	sc_inout <sc_uint<8> > r_currentPositionArray_port[3];
	sc_inout <sc_uint<8> > r_nextPositionArray_port[3];

	/*sc_inout <sc_uint<8> > o_currentPositionArray[MAX_OBSTACLES];
	sc_inout <sc_uint<8> > o_nextPositionArray[MAX_OBSTACLES];*/
	
	//sc_inout<sc_uint<8> > robot


	sc_uint<8>  r_IndexArray[ENVIRONMENT_MAX_ROBOTS];
	sc_uint<8>  r_currentGridArray[ENVIRONMENT_MAX_ROBOTS];
	sc_uint<8>  r_nextGridArray[ENVIRONMENT_MAX_ROBOTS];
	//sc_uint<8>	r_xArray[MAX_ROBOTS];
	//sc_uint<8>	r_yArray[MAX_ROBOTS];
	sc_uint<8>  r_statusArray[ENVIRONMENT_MAX_ROBOTS];
	sc_uint<8>  r_robotArray[ENVIRONMENT_MAX_ROBOTS][ROBOT_NUM_COLUMNS];



	sc_uint<8>  o_IndexArray[ENVIRONMENT_MAX_OBSTACLES];
	sc_uint<8>  o_currentGridArray[ENVIRONMENT_MAX_OBSTACLES];
	sc_uint<8>  o_nextGridArray[ENVIRONMENT_MAX_OBSTACLES];
	//sc_uint<8>	o_xArray[MAX_OBSTACLES];
	//sc_uint<8>	o_yArray[MAX_ROBOTS];
	sc_uint<8>  o_obstacleArray[ENVIRONMENT_MAX_OBSTACLES][OBSTACLE_NUM_COLUMNS];

	SC_HAS_PROCESS(ENVIRONMENT);


	ENVIRONMENT(sc_module_name name, const T* numRobots, const T* numObstacles) : sc_module(name), _numRobots(numRobots), _numObstacles(numObstacles)
	{

		cout << "CREATING ENVIRONMENT..." << "\tName: " << name << "\t# of Robots: " << *(_numRobots) << "\t# of Obstacles: " << *(_numObstacles) << endl;

		SC_CTHREAD(run1, clock.pos());
		SC_METHOD(prc_environment);
		sensitive << clock.pos();

	}
	void prc_environment(){

		// FOR EACH ROBOT, UPDATE POSITION
		for (int i = 0; i < *(_numRobots); i++){
			r_IndexArray[i] = r_robotIDArray_port[i].read();
			r_currentGridArray[i] = r_currentPositionArray_port[i].read();
			r_nextGridArray[i] = r_nextPositionArray_port[i].read();
			r_statusArray[i] = r_robotStatus_port[i].read();

		}

		// FOR EACH OBSTACLE, UPDATE POSITION
		//for (int i = 0; i < *(_numObstacles); i++){
				o_IndexArray[0] = 0;
				o_currentGridArray[0] = 25;
				o_nextGridArray[0] = 25;
			//}
			//else if (i ==1)
				o_IndexArray[1] = 1;
				o_currentGridArray[1] = 60;
				o_nextGridArray[1] = 60;
			
		//}

		// FOR EACH ROBOT, CHECK IF ROBOT IS CLOSE TO BOUNDARY/OTHER ROBOTS
		for (int i = 0; i < *(_numRobots); i++){

			//int thisRobotCurrentGrid = r_currentGridArray[i];

			if (r_statusArray[i] == 1){
				r_environmentStatus_port[i].write(1);
			}
			else{
				r_environmentStatus_port[i].write(0);
			}
		}

		//FOR EACH ROBOT, COMPARE WITH OBSTACLES
		for (int i = 0; i < *(_numRobots); i++){
			int obstaclesInFront = 0;

			int thisRobotSpeed = r_nextGridArray[i] - r_currentGridArray[i]; //5

			/*for (int j = 0; j < *(_numObstacles); j++){
				if (r_nextGridArray[i] < o_currentGridArray[j]){
					obstaclesInFront++;
				}
			}*/

			for (int j = 0; j < *(_numObstacles); j++){

				// CHECK ONLY OBJECTS IN FRONT OF GRID
				if (r_currentGridArray[i] < o_currentGridArray[j] && r_nextGridArray[i] >= o_currentGridArray[j]){
					r_environmentStatus_port[i].write(0);
				}
				else{
					r_environmentStatus_port[i].write(1);
				}
					//int temp = o_currentGridArray[j] - r_currentGridArray[i];

					//// within range
					//if (temp <= (r_nextGridArray[i] - r_currentGridArray[i]) && (temp > 0)){
					
			}
				/*	++;
				}
				else if (obstaclesInFront = 0){
					r_environmentStatus_port[i].write(1);
				}*/

		}
				// IF THIS ROBOT IS WITHIN THIS ROBOT's NEXT GRID of an obstacle, tell the robot to stop
				/*int distanceToObstacle = o_currentGridArray[j] - r_nextGridArray[i];

				if (distanceToObstacle < 0){
					distanceToObstacle = distanceToObstacle * (-1);
				}
				*/		

				//// THIS OBJECT IS BEHIND THE ROBOT, IGNORE
				//if (o_currentGridArray[j] <= r_currentGridArray[i]){
				//	break;
				//}
				
				//if (r_nextGridArray[i] <= o_currentGridArray[j]){
				//	r_environmentStatus_port[i].write(0);
				//}
				//else{

				//}
			/*	if (r_nextGridArray[i] - o_currentGridArray[j] <=  thisRobotSpeed){*/

				//if (n[j] - r_currentGridArray[i] < (r_nextGridArray[i] - r_currentGridArray[i])){
				//if (r_nextGridArray[i] >= o_currentGridArrayy[j]){
				//if (r_nextGridArray[i] <= (o_currentGridArray[j] + thisRobotSpeed) ){

				
				

				/*if (o_currentGridArray[j] <= )*/
			

		cout << endl << "~~~~~~~~~~~ENVIRONMENT~~~~~~~~~~~" << endl;
		cout << endl << "===========ROBOT ARRAY===========" << endl;
		cout << "|RI\t|CG\t|NG\t|STATUS\t|" << endl;
		for (int i = 0; i < *(_numRobots); i++){

			r_robotArray[i][0] = r_IndexArray[i];
			r_robotArray[i][1] = r_currentGridArray[i];
			r_robotArray[i][2] = r_nextGridArray[i];
			//r_robotArray[i][3] = xArray[i];
			//r_robotArray[i][4] = yArray[i];
			r_robotArray[i][3] = r_statusArray[i];
			cout << "|";
			for (int j = 0; j < ROBOT_NUM_COLUMNS; j++){
				cout << r_robotArray[i][j] << "\t|";
			}
			cout << endl;
		}
		cout << "=================================" << endl;

		cout << endl << "======OBSTACLE ARRAY=====" << endl;
		cout << "|OI\t|CG\t|NG\t|" << endl;
		for (int i = 0; i < NUM_OBSTACLES; i++){

			o_obstacleArray[i][0] = o_IndexArray[i];
			o_obstacleArray[i][1] = o_currentGridArray[i];
			o_obstacleArray[i][2] = o_nextGridArray[i];
			//o_obstacleArray[i][3] = xArray[i];
			//o_obstacleArray[i][4] = yArray[i];
			cout << "|";
			for (int j = 0; j < OBSTACLE_NUM_COLUMNS; j++){
				cout << o_obstacleArray[i][j] << "\t|";
			}
			cout << endl;
		}
		cout << "=========================" << endl;
	}
	void run1(){
		wait();
		cout << endl << "~~~~~~~~~~1st CLOCK ENVIRONMENT~~~~~~~~~ "<< endl;
		/*r_environmentStatus_port[0].write(1);
		r_environmentStatus_port[1].write(1);
		r_environmentStatus_port[2].write(1);*/
		wait();


		cout << endl << "~~~~~~~~~2nd CLOCK ENVIRONMENT~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~3rd CLOCK ENVIRONMENT~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~4th CLOCK ENVIRONMENT~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~5th CLOCK ENVIRONMENT~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~6th CLOCK ENVIRONMENT~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~7th CLOCK ENVIRONMENT~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~8th CLOCK ENVIRONMENT~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~9th CLOCK ENVIRONMENT~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~10th CLOCK ENVIRONMENT~~~~ "<< endl;
		wait();


	}
private:
	const T* _numRobots;
	const T* _numObstacles;
};
