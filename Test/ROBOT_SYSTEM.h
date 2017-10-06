#include "systemc.h"
#include "ROBOT.h"
#define _CRT_SECURE_NO_WARNINGS

SC_MODULE(ROBOT_SYSTEM){

	//// MODULES TO CONNECT
	//typedef int robot_T;
	////const sc_module_name name = "r1";
	//const robot_T speed = 0;
	//const robot_T path = 0;

	//ROBOT<robot_T>	*r1("r1");
	////ROBOT<robot_T> *r1;
	////ROBOT<r1_T> *r1(const sc_module_name, const r1_T, const r1_T);
	////ROBOT<const r1_T> *r1;

	//// DEFINE SIGNALS
	//sc_clock					clk_sig;
	//sc_signal<sc_uint<8> >		in1_sig;
	//sc_signal<sc_uint<8> >		in2_sig;
	//sc_signal<sc_uint<8> >		in3_sig;
	//sc_signal<bool>				load1_sig;
	//sc_signal<bool>				load2_sig;
	//sc_signal<bool>				dec1_sig;
	//sc_signal<bool>				dec2_sig;
	//sc_signal<sc_uint<8> >		count1_sig;
	//sc_signal<sc_uint<8> >		count2_sig;
	//sc_signal<bool>				ended_sig;

	//// CONSTRUCTOR
	//SC_CTOR(ROBOT_SYSTEM) : clk_sig("clk_sig", 10, SC_NS)
	//{
	//	r1.clock(clk_sig);
	//	//r1.clock(clk_sig);
	//	//r1->clock(clk_sig);
	//	//cout << "something";
	//	//r1->clock(clk_sig);
	//	sc_start(500, SC_NS);
	//}

	//// DECONSTRUCTOR
	//~ROBOT_SYSTEM(){
	//	delete r1;
	//}

};