#include "disp.h"

void DispfromFC()
{
	#ifndef FC_DEBUG_MODE
		cout << "From FlightCtrl \t" << from_fc.accelerometer[0] << "\t" << from_fc.accelerometer[1] << "\t" << from_fc.accelerometer[2] << endl;	
	#else
		cout << "From FlightCtrl (Debug) \t";
		cout << for_debug.motor_setpoint[0] << "\t" << for_debug.motor_setpoint[1] << endl;
	#endif
}