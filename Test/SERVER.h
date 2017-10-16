#include "systemc.h"
#define _CRT_SECURE_NO_WARNINGS

const int SERVER_MAX_ROBOTS = 5;
const int NUM_COLUMNS = 3;
const int S_NUM_ROBOTS = 2;

template <class T> class SERVER : public sc_module {

public:

	sc_in<bool> clock;
	

	sc_in <sc_uint<8> > r_id_port[S_NUM_ROBOTS];			// USED TO IDENTIFY EACH ROBOT
	sc_out<bool>	gridUpdate_port[S_NUM_ROBOTS];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
	sc_in<bool>		obstacle_port[S_NUM_ROBOTS];			// Server(in) <-- ROBOT(inout) <-- Environment(out)
	sc_in<bool>		boundary_port[S_NUM_ROBOTS];			// Server(in) <-- ROBOT(inout) <-- Environment(out)
	
	sc_int<8>  r_index_array[S_NUM_ROBOTS];					// COLUMN FOR EACH ROBOT INDEX
	sc_int<8>  r_cg_array[S_NUM_ROBOTS];					// COLUMN FOR EACH ROBOT's CURRENT GRID
	sc_int<8>  r_ng_array[S_NUM_ROBOTS];					// COLUMN FOR EACH ROBOT's NEXT GRID
	sc_int<8>  r_status_array[S_NUM_ROBOTS];				// COLUMN FOR EACH ROBOT's STATUS

	sc_int<8>  server_array[S_NUM_ROBOTS][NUM_COLUMNS];	// SERVER DATA STRUCTURE 
	
	void prc_server();

	SC_HAS_PROCESS(SERVER);

	SERVER(sc_module_name name, const T* numRobots, const T* r0_id, const T* r0_speed, const T* r0_grid, const T* r1_id, const T* r1_speed, const T* r1_grid) :
		sc_module(name), _numRobots(numRobots), _r0_id(r0_id), _r0_speed(r0_speed), _r0_grid(r0_grid), _r1_id(r1_id), _r1_speed(r1_speed), _r1_grid(r1_grid)
	{
		r_index_array[*(r0_id)] = *(r0_id);
		r_cg_array[*(r0_id)] = *(r0_grid);
		r_ng_array[*(r0_id)] = *(r0_grid)+1;
		r_status_array[*(r0_id)] = 1;

		r_index_array[*(r1_id)] = *(r1_id);
		r_cg_array[*(r1_id)] = *(r1_grid);
		r_ng_array[*(r1_id)] = *(r1_grid)+1;
		r_status_array[*(r1_id)] = 1;

		for (int i = 0; i < *(_numRobots); i++){
			server_array[i][0] = r_index_array[i];
			server_array[i][1] = r_cg_array[i];
			server_array[i][2] = r_ng_array[i];
			server_array[i][3] = r_status_array[i];
		}


		cout << "CREATING SERVER..." << "\tName: " << name << "\t# of Robots: " << *(_numRobots) << endl;
		SC_CTHREAD(run, clock.pos());
		SC_METHOD(prc_server);
		sensitive << clock.pos();
		
	}
	
	


	void run(){
		//cout << endl << "~~~~~~~~~~~~~~~~~~~~1st CLOCK SERVER~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		///*serverStatus_port[0].write(1);
		//serverStatus_port[1].write(1);
		//serverStatus_port[2].write(1);*/
		//
		//cout << endl << "~~~~~~~~~~~~~~~~~~~~2nd CLOCK SERVER~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<< endl;
		//wait();
		//
		//cout << endl << "~~~~~~~~~~~~~~~~~~~~3rd CLOCK SERVER~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<< endl;
		//wait();
		//
		//cout << endl << "~~~~~~~~~~~~~~~~~~~~4th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~5th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~6th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~7th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~8th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~9th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~10th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~11th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~12th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~13th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~14th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~15th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~16th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~17th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~18th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~19th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();

		//cout << endl << "~~~~~~~~~~~~~~~~~~~~20th CLOCK ENVIRONMENT~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		//wait();


		
	}

private:
	const T* _numRobots;

	const T* _r0_id;
	const T* _r0_speed;
	const T* _r0_grid;

	const T* _r1_id;
	const T* _r1_speed;
	const T* _r1_grid;
};
