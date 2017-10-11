#include "systemc.h"
#define _CRT_SECURE_NO_WARNINGS



template <class T> class ROBOT : public sc_module{

public:
	//	INPUT SIGNALS
	sc_in<bool> clock;		
	sc_out <sc_uint<8> > id_port;	// USED TO IDENTIFY THIS ROBOT
	sc_in<bool> s_status_port;		// USED TO READ CONTROL FROM SERVER
	sc_out<bool> r_status_port;		// USED TO TRANSFER STATUSES BETWEEN SERVER AND ENVIRONMENT
	sc_in<bool> e_status_port;		// USED TO READ CONTROL FROM ENVIRONMENT 


	SC_HAS_PROCESS(ROBOT);
	ROBOT(sc_module_name name, const T* id, const T* speed, T* grid, T* x, T* y) : sc_module(name), _id(id), _speed(speed), _grid(grid), _x(x), _y(y)
	{
		SC_METHOD(prc_robot);
		sensitive << clock.pos();
		cout << "CREATING ROBOT..." << "\tName: " << name << "\tROBOT ID: " << *(_id) << "\tSpeed: " << *(_speed) << "\Grid: " << *(_grid) << "\tX: " << *(_x) << "\tY: " << *(_y) << endl;
		
	}
	void prc_robot(){
		id_port.write(*(_id));

		// CHECK STATUS

		// ENVIRONMENT SAID YOU CAN MOVE
		if (e_status_port.read() == 1){

			// SERVER SAID YOU CAN MOVE
			if (s_status_port.read() == 1){
				r_status_port.write(1);
			}
			// SERVER SAID YOU CANT MOVE
			else{
				r_status_port.write(0);
			}

		}

		// ENVIRONMENT SAID YOU CANT MOVE
		else{
			r_status_port.write(0);

		}



		cout << endl;
		cout << "~~~~~~~~~~~~~ROBOT " << id_port.read() << "~~~~~~~~~~~~~" << endl;
		cout << "=================================" << endl;
		cout << "| Robot Status:     " << r_status_port.read() << "\t\t|" << endl;
		cout << "| Server Status:    " << s_status_port.read() << "\t\t|" << endl;
		cout << "| Env Status:       " << e_status_port.read() << "\t\t|" << endl;
		cout << "=================================" << endl;
	}

	void run(){
		
	}

private:
	const T* _id;
	const T* _speed;
	T* _grid;
	T* _x;
	T* _y;


	};
