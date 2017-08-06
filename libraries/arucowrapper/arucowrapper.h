#ifndef MYMARKER_H_
#define MYMARKER_H_

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <cstring>
#include <sys/time.h>

using namespace std;
using namespace cv;
using namespace aruco;
using namespace Eigen;

// =============================================================================
// Parameters:

/* mount offset [m] */
#define OFFSET_X 0
#define OFFSET_Y 0

/* marker packet */
struct Packet {
  uint32_t timestamp; // microseconds
  float position[3]; // meter
  float quaternion[3]; // x y z
  float r_var[3]; // meter^2
  uint8_t status; // 1 : detected, 0 : not detected
} __attribute__((packed));

static ofstream fout("../output_data/state.csv", ios::out);

// =============================================================================

class aruco_wrapper
{
private:
  MarkerMap MM;
  CameraParameters CP;
  MarkerMapPoseTracker MMPT;
  struct timeval tv;
  struct Packet packet;
  void getpos(Mat Rvec, Mat Tvec, float position[3]);
  void geterr_(Marker marker, Mat Rvec, Mat Tvec, float err_[3]);
  void geterr(vector<Marker> v_m, Mat Rvec, Mat Tvec, float err[3]);
  void getquaternion(Mat Rvec, float quaternion[3]);
  bool checkmarker(vector<Marker> v_m);
  bool checkpos(float position[3], float dt);
public:
  aruco_wrapper(string pathforCP, string pathforMM);
  ~aruco_wrapper() {};
  void SetCameraParameters(string filepath);
  void SetMarkerMap(string filepath);
  void SetMarkerMapPoseTracker();
  bool MarkerUpdate(Mat img);
  void Disp();
  void Logging();
  struct Packet* Packet();
};

#endif // MYMARKER_H_