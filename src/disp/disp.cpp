#include "disp.h"

void DispfromFC()
{
	#ifndef FC_DEBUG_MODE
		cout << "From FlightCtrl \t" << from_fc.accelerometer[0] << "\t" << from_fc.accelerometer[1] << "\t" << from_fc.accelerometer[2] << endl;	
	#else
		cout << "From FlightCtrl (Debug) \t" << endl;
	#endif
}