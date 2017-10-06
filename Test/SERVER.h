#include "systemc.h"
#define _CRT_SECURE_NO_WARNINGS

const int SERVER_MAX_ROBOTS = 5;
const int NUM_COLUMNS = 4;

template <class T> class SERVER : public sc_module{

public:

	sc_in<bool> clock;
	

	sc_in <sc_uint<8> > robotIDArray_port[3];
	sc_in<sc_uint<8> > currentPositionArray_port[3];
	sc_in<sc_uint<8> > nextPositionArray_port[3];
	

	sc_in<bool> robotStatus_port[3];
	sc_out<bool> serverStatus_port[3];


	sc_uint<8>  robotIndexArray[SERVER_MAX_ROBOTS];
	sc_uint<8>  currentGridArray[SERVER_MAX_ROBOTS];
	sc_uint<8>  nextGridArray[SERVER_MAX_ROBOTS];
	sc_uint<8>  statusArray[SERVER_MAX_ROBOTS];
	sc_uint<8>  serverArray[SERVER_MAX_ROBOTS][NUM_COLUMNS];
	SC_HAS_PROCESS(SERVER);


	SERVER(sc_module_name name, const T* numRobots) : sc_module(name), _numRobots(numRobots)
	{

		cout << "CREATING SERVER..." << "\tName: " << name << "\t# of Robots: " << *(_numRobots) << endl;
		SC_CTHREAD(run, clock.pos());
		SC_METHOD(prc_server);
		sensitive << clock.pos();
	
	}
	void prc_server(){

		// UPDATE DATA STRUCTURE
		for (int i = 0; i < *(_numRobots); i++){
			robotIndexArray[i] = robotIDArray_port[i].read();
			statusArray[i] = robotStatus_port[i].read();
		}

		// FOR ALL ROBOTS
		for (int i = 0; i < *(_numRobots); i++){

			// GET THIS ROBOTS CURRENT AND NEXT GRID POSITION
			currentGridArray[i] = currentPositionArray_port[i].read();
			nextGridArray[i] = nextPositionArray_port[i].read();

			// FOR EACH STOPPED ROBOT
			if (statusArray[i] == 0){
				//cout << "ROBOT " << i << " STOPPED" << endl;

				// CHECK IF THIS ROBOT'S NEXTGRID IS EQUAL TO ANY ROBOTS CURRENTGRID
				for (int j = 0; j < *(_numRobots); j++){

					if (j != i){
						// TELL ROBOT TO STOP IF SO
						if (nextGridArray[i] == currentGridArray[j]){
							//cout << "ROBOT " << i << " NEXT GRID EQUALS ROBOT " << j << " CURRENT GRID..MUST STOP" << endl;
							serverStatus_port[j].write(0);
						}
						// TELL ROBOT IT CAN MOVE IF NOT
						else{
							//cout << "ROBOT " << i << " NEXT GRID NOT EQUAL OTHER ROBOTS...CAN CONTINUE " << endl;
							serverStatus_port[j].write(1);
						}
					}
				}
			}
		}

		cout << endl << "~~~~~~~~~~~~~Server~~~~~~~~~~~~~~" << endl;
		cout << "=================================" << endl;
		// THEN READ DATA STRUCTURE FROM LOCAL ARRAYS
		cout << "|RI\t|CG\t|NG\t|STATUS\t|" << endl;
		for (int i = 0; i < *(_numRobots); i++){
			serverArray[i][0] = robotIndexArray[i];
			serverArray[i][1] = currentGridArray[i];
			serverArray[i][2] = nextGridArray[i];
			serverArray[i][3] = statusArray[i];

			cout << "|";
			for (int j = 0; j < NUM_COLUMNS; j++){
				cout << serverArray[i][j] << "\t|";
			}
			cout << endl;
		}
		cout << "=================================" << endl;
	}


	void run(){
		wait();
		cout << endl << "~~~~~~~~~1st CLOCK SERVER~~~~~~~~~" << endl;
		/*serverStatus_port[0].write(1);
		serverStatus_port[1].write(1);
		serverStatus_port[2].write(1);*/
		wait();
		
		cout << endl << "~~~~~~~~~2nd CLOCK SERVER~~~~~~~~~ "<< endl;
		wait();
		
		cout << endl << "~~~~~~~~~3rd CLOCK SERVER~~~~~~~~~ "<< endl;
		wait();
		
		cout << endl << "~~~~~~~~~4th CLOCK SERVER~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~5th CLOCK SERVER~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~6th CLOCK SERVER~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~7th CLOCK SERVER~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~8th CLOCK SERVER~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "~~~~~~~~~9th CLOCK SERVER~~~~~~~~~ "<< endl;
		wait();

		cout << endl << "-~~~~~~~~~10th CLOCK SERVER~~~~~~~~~ "<< endl;
		wait();

		
	}

private:
	const T* _numRobots;
};
