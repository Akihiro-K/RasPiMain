#ifndef LOGGER_H_
#define LOGGER_H_

#include "../shared/shared.h"
#include <fstream>
#include <sys/time.h>

using namespace std;

static struct timeval tv;
static ofstream fout("../output_data/log.csv", ios::out);
static ofstream fout2("../output_data/tofc.csv", ios::out);

void InitLogging();

void FCLogging();

void ToFCLogging();

void VisionLogging();

void GPSLogging();

void LSMLogging();

#endif
