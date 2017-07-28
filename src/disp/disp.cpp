#include "disp.h"

void DispfromFC()
{
	cout << "From FlightCtrl \t" << from_fc.accelerometer[0] << "\t" << from_fc.accelerometer[1] << "\t" << from_fc.accelerometer[2] << endl;	
}

void DispfromFCforDebug()
{
	cout << "From FlightCtrl (Debug) \t" << endl;
}