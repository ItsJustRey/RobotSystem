#include "systemc.h"
#define _CRT_SECURE_NO_WARNINGS

const int SERVER_MAX_ROBOTS = 5;
const int NUM_COLUMNS = 4;
const int S_NUM_ROBOTS = 2;

template <class T> class SERVER : public sc_module{

public:

	sc_in<bool> clock;
	

	sc_in <sc_uint<8> > r_id_port[S_NUM_ROBOTS];			// USED TO IDENTIFY EACH ROBOT

	sc_in<bool> r_status_port[S_NUM_ROBOTS];				// USED TO READ CONTROL FROM EACH ROBOT 
	sc_out<bool> s_status_port[S_NUM_ROBOTS];				// USED TO TRANSFER CONTROL TO EACH ROBOT


	sc_uint<8>  r_index_array[SERVER_MAX_ROBOTS];				// COLUMN FOR EACH ROBOT INDEX
	sc_uint<8>  r_cg_array[SERVER_MAX_ROBOTS];					// COLUMN FOR EACH ROBOT's CURRENT GRID
	sc_uint<8>  r_ng_array[SERVER_MAX_ROBOTS];					// COLUMN FOR EACH ROBOT's NEXT GRID
	sc_uint<8>  r_status_array[SERVER_MAX_ROBOTS];				// COLUMN FOR EACH ROBOT's STATUS


	sc_uint<8>  server_array[SERVER_MAX_ROBOTS][NUM_COLUMNS];	// SERVER DATA STRUCTURE 


	SC_HAS_PROCESS(SERVER);
	SERVER(sc_module_name name, const T* numRobots, const T* r0_id, const T* r0_speed, const T* r0_grid, const T* r1_id, const T* r1_speed, const T* r1_grid) :
		sc_module(name), _numRobots(numRobots), _r0_id(r0_id), _r0_speed(r0_speed), _r0_grid(r0_grid), _r1_id(r1_id), _r1_speed(r1_speed), _r1_grid(r1_grid)
	{

		cout << "CREATING SERVER..." << "\tName: " << name << "\t# of Robots: " << *(_numRobots) << endl;
		SC_CTHREAD(run, clock.pos());
		SC_METHOD(prc_server);
		sensitive << clock.pos();
	
	}
	void prc_server(){

		// UPDATE DATA STRUCTURE
		for (int i = 0; i < *(_numRobots); i++){
			r_index_array[i] = r_id_port[i].read();
			r_status_array[i] = r_status_port[i].read();
		}

		//// FOR ALL ROBOTS
		


		cout << endl << "~~~~~~~~~~~~~Server~~~~~~~~~~~~~~" << endl;
		cout << "=================================" << endl;
		// THEN READ DATA STRUCTURE FROM LOCAL ARRAYS
		cout << "|RI\t|CG\t|NG\t|STATUS\t|" << endl;
		for (int i = 0; i < *(_numRobots); i++){
			server_array[i][0] = r_index_array[i];
			server_array[i][1] = r_cg_array[i];
			server_array[i][2] = r_ng_array[i];
			server_array[i][3] = r_status_array[i];

			cout << "|";
			for (int j = 0; j < NUM_COLUMNS; j++){
				cout << server_array[i][j] << "\t|";
			}
			cout << endl;
		}
		cout << "=================================" << endl;
	}


	void run(){
		wait();
		cout << endl << "~~~~~~~~~~~~~~~~~~~~1st CLOCK SERVER~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
		/*serverStatus_port[0].write(1);
		serverStatus_port[1].write(1);
		serverStatus_port[2].write(1);*/
		wait();
		
		cout << endl << "~~~~~~~~~~~~~~~~~~~~2nd CLOCK SERVER~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<< endl;
		wait();
		
		cout << endl << "~~~~~~~~~~~~~~~~~~~~3rd CLOCK SERVER~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<< endl;
		wait();
		
		cout << endl << "~~~~~~~~~~~~~~~~~~~~4th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~5th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~" << endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~6th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~7th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~8th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~9th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~~~~~~~~~~~~10th CLOCK SERVER~~~~~~~~~~~~~~~~~~~~ "<< endl;
		wait();

		
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
