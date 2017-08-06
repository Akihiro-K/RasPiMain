#ifndef LOGGER_H_
#define LOGGER_H_

#include "../shared/shared.h"
#include <fstream>
#include <sys/time.h>

using namespace std;

static struct timeval tv;
static ofstream fout("../log.csv", ios::out);
static ofstream fout2("../tofc.csv", ios::out);

void InitLogging();

void FCLogging();

void ToFCLogging();

void VisionLogging();

void GPSLogging();

void LSMLogging();

#endif
