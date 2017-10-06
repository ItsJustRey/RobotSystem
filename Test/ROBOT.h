#include "systemc.h"
#define _CRT_SECURE_NO_WARNINGS



template <class T> class ROBOT : public sc_module{

public:
	//	INPUT SIGNALS
	sc_in<bool> clock;
	sc_in<bool> serverStatus_port;
	sc_in<bool> environmentStatus_port;
	sc_inout<bool> robotStatus_port;
	sc_out <sc_uint<8> > robotID;

	

	sc_inout <sc_uint<8> > currentPositionArray;
	sc_inout <sc_uint<8> > nextPositionArray;


	sc_signal<bool>  robotStatus;
	sc_signal<bool>  serverStatus;
	sc_signal<bool>  environmentStatus;

	SC_HAS_PROCESS(ROBOT);

	int runspeed = 0;
	
		
	ROBOT(sc_module_name name, const T* robotID, const T* speed, T* position) : sc_module(name), _robotID(robotID), _speed(speed), _position(position)
	{
		//currentPosition = *(_position);
		SC_METHOD(prc_robot);
		//SC_THREAD(run);
		sensitive << clock.pos();
		cout << "CREATING ROBOT..." << "\tName: " << name << "\tROBOT ID: " << *(_robotID) << "\tSpeed: " << *(_speed) << "\tPosition: " << *(_position) << endl;
	}
	void prc_robot(){
		robotID.write(*(_robotID));
		*(_position) = *(_position);

		serverStatus = serverStatus_port.read();
		environmentStatus = environmentStatus_port.read();
		robotStatus = robotStatus_port.read();
		// CHECK STATUS

		// ENVIRONMENT SAID YOU CAN MOVE
		if (environmentStatus == 1){
			//cout << "ROBOT " << *(_robotID) << " CAN MOVE (ENVIRONMENT STATUS = 1) " << endl;
			// SERVER SAID YOU CAN MOVE
			if (serverStatus == 1){
				//cout << "ROBOT " << *(_robotID) << " CAN MOVE (SERVER STATUS = 1) " << endl;
				robotStatus_port.write(1);
				*_position = *(_position)+*(_speed);
			}
			// SERVER SAID YOU CANT MOVE
			else{
				//cout << "ROBOT " << *(_robotID) << " CANT MOVE (SERVER STATUS = 0) " << endl;
				robotStatus_port.write(0);
				*_position = *(_position);
			}

		}

		// ENVIRONMENT SAID YOU CANT MOVE
		else{
			//cout << "ROBOT " << *(_robotID) << " CAN MOVE (ENVIRONMENT STATUS = 1) " << endl;
			robotStatus_port.write(0);
			*_position = *(_position);

		}

		currentPositionArray.write(*(_position));
		nextPositionArray.write(*(_position)+*(_speed));


		cout << endl;

		cout << "~~~~~~~~~~~~~ROBOT " << robotID.read() << "~~~~~~~~~~~~~" << endl;
		cout << "=================================" << endl;
		cout << "| Speed:            " << *(_speed) << "m/s\t|" << endl;
		cout << "| Current Position: " << currentPositionArray.read() << "m\t\t|" << endl;
		//cout << "| Current Position PARAMETER: " << *(_position) << "m\t\t|" << endl;
		/*cout << "| Robot Status:     " << robotStatus << "\t\t|" << endl;
		cout << "| Server Status:    " << (serverStatus && environmentStatus) << "\t\t|" << endl;
		cout << "| Env Status:       " << (serverStatus && environmentStatus) << "\t\t|" << endl;*/
		cout << "=================================" << endl;
	}

	void run(){
		/*while (1){
			if (status.read() == 1){
				cout << "DETECTED Status: " << status.read() << endl;
				*_position = *(_position)+*(_speed);
			}
			else{
				cout << "DONT MOVE Status: " << status.read() << endl;
			}

			cout << endl;
			cout << "--------------------------------" << endl;
			cout << "-------------ROBOT------------" << endl;
			cout << "Speed:\t" << *(_speed) << endl;
			cout << "Position:\t" << *(_position) << endl;
			wait();
		}*/
		/*wait();
		

		

		cout << endl;
		cout << "--------------------------------" << endl;
		cout << "-------------ROBOT------------" << endl;
		cout << "Speed:\t" << *(_speed) << endl;
		cout << "Position:\t" << *(_position) << endl;*/

/*
		wait();
		wait();
		wait();
		wait();
		wait();
		wait();*/
		//while (1){

		//	//*(_position) += *(_speed);


		//}




		//cout << "Location ARRAY:\t" << locationArray.read() << endl;
		
	}

private:
	const T* _speed;
	T* _position;
	const T* _robotID;


	};
