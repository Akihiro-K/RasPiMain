#ifndef MYMARKER_H_
#define MYMARKER_H_

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>

#include <cmath>

#include <time.h>
#include <sys/time.h>

using namespace cv;
using namespace aruco;
using namespace Eigen;

// =============================================================================
// Parameters:

/* mount angle [deg] */
#define ANGLE 0

/* deg to meter */
// Note:
// Latitude is 35.7090 [deg]
#define LAT_TO_MET 111263.283
#define LON_TO_MET 90344.879

/* number of ports */
#define PORT_NUM 2
#define HOME 2

/* position of drone ports [deg] */
// Note:
// Port 1 is take-off port(Home).
// Port 2 is landing port.
static float LAT[] = {35.499170, 35.499283};
static float LON[] = {139.421927, 139.422436};

/* marker packet */
struct STATE {
  uint32_t timestamp; // microseconds
  float position[3]; // meter
  float quaternion[3]; // x y z
  float r_var[3]; // meter^2
  uint8_t status; // 1 : detected, 0 : not detected
} __attribute__((packed));

// =============================================================================



// =============================================================================
// Private functions:

void getpos(Mat Rvec, Mat Tvec, float position[3], int port_num);

void geterr_(Marker marker, MarkerMap MM, Mat Rvec, Mat Tvec, CameraParameters CP, float err_[3]);

void geterr(vector<Marker> v_m, MarkerMap MM, Mat Rvec, Mat Tvec, CameraParameters CP, float err[3]);

void getquaternion(Mat Rvec, float quat[3]);

bool checkmarker(vector<Marker> v_m, vector<MarkerMap> MMs, int &num);

bool checkpos(float position[3], float dt, MarkerMap MM, CameraParameters CP, MarkerMapPoseTracker &MMPT);

// =============================================================================

// =============================================================================
// Public functions:
bool marker_mes(Mat img, vector<MarkerMapPoseTracker> &MMPTs, vector<MarkerMap> MMs, CameraParameters CP, struct timeval &tv, struct STATE &S);

void no_marker(struct timeval &tv, struct STATE &S);

void savetofile(struct STATE S, ofstream &fout);
// =============================================================================

#endif // MYMARKER_H_
