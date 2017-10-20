#include "systemc.h"
#define _CRT_SECURE_NO_WARNINGS



template <class T> class ROBOT : public sc_module{

public:
	//	INPUT SIGNALS
	sc_in<bool> clock;
	sc_out <sc_uint<8> > id_port;	// USED TO IDENTIFY THIS ROBOT

	sc_inout<bool>gridUpdate_port;		// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
	sc_inout<bool>obstacle_port;		// Server(in) <-- ROBOT(inout) <-- Environment(out)
	sc_inout<bool>boundary_port;		// Server(in) <-- ROBOT(inout) <-- Environment(out)

	sc_inout<bool>	robot_start_moving_port;		// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)

	sc_inout<sc_int<8> >e_cg_array_port;
	sc_inout<sc_int<8> >e_ng_array_port;

	//
	sc_inout<sc_int<8> >	block_array_port_inout[8];			// Server(out)-- >ROBOT(inout)--> ENVIRONMENT(in)
	sc_int<8> block_array[8];

	void prc_robot();
	void print_robot();

	SC_HAS_PROCESS(ROBOT);
	ROBOT(sc_module_name name, const T* id, const T* speed, T* grid, T* x, T* y) : sc_module(name), _id(id), _speed(speed), _grid(grid), _x(x), _y(y)
	{
		SC_METHOD(prc_robot);
		sensitive << clock.pos();
		dont_initialize();
		SC_METHOD(print_robot);
		sensitive << clock.pos();
		dont_initialize();
		cout << "CREATING ROBOT..." << "\tName: " << name << "\tROBOT ID: " << *(_id) << "\tSpeed: " << *(_speed) << "\Grid: " << *(_grid) << "\tX: " << *(_x) << "\tY: " << *(_y) << endl;

	}

private:
	const T* _id;
	const T* _speed;
	T* _grid;
	T* _x;
	T* _y;


};