#include "arucowrapper.h"

// =============================================================================
// Private functions:

void aruco_wrapper::getpos(Mat Rvec, Mat Tvec, float position[3]) 
{
  Mat pos, rmat;
  Rodrigues(Rvec, rmat);
  pos = -rmat.inv() * Tvec.t();
  position[0] = pos.at<float>(0,0) - OFFSET_X;
  position[1] = pos.at<float>(0,1) - OFFSET_Y;
  position[2] = pos.at<float>(0,2);
}

void aruco_wrapper::geterr_(Marker marker, Mat Rvec, Mat Tvec, float err_[3])
{
  // get 3Dinfo from marker map
  Marker3DInfo a = MM.getMarker3DInfo(marker.id);
  
  // Intrinsic parameters
  Matrix3f K;
  K << CP.CameraMatrix.at<float>(0,0), CP.CameraMatrix.at<float>(0,1), CP.CameraMatrix.at<float>(0,2),
     CP.CameraMatrix.at<float>(1,0), CP.CameraMatrix.at<float>(1,1), CP.CameraMatrix.at<float>(1,2),
     CP.CameraMatrix.at<float>(2,0), CP.CameraMatrix.at<float>(2,1), CP.CameraMatrix.at<float>(2,2);
     
  // Rotation Matrix
  Mat rmat, dr_drmat;
  Rodrigues(Rvec, rmat, dr_drmat);
  Matrix3f R;
  R << rmat.at<float>(0,0), rmat.at<float>(0,1), rmat.at<float>(0,2),
     rmat.at<float>(1,0), rmat.at<float>(1,1), rmat.at<float>(1,2),
     rmat.at<float>(2,0), rmat.at<float>(2,1), rmat.at<float>(2,2);	
  
  // Translation Vector
  Vector3f t0(Tvec.at<float>(0,0), Tvec.at<float>(0,1), Tvec.at<float>(0,2));

  /* calculate Jacobian */
  Matrix<float, 8, 6> A;
  Vector3f P, P_;
  float dp_dr[9];
  float fx = CP.CameraMatrix.at<float>(0,0);
  float fy = CP.CameraMatrix.at<float>(1,1);
  
  // corner 0
  P << a[0].x, a[0].y, a[0].z;
  P_ = R * P + t0;
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      dp_dr[j*3+i] = dr_drmat.at<float>(j,i*3+0) * P[0] + dr_drmat.at<float>(j,i*3+1) * P[1] + dr_drmat.at<float>(j,i*3+2) * P[2];
    }
  }
  A(0,0) = fx * (dp_dr[0] / P_[2] - P_[0] * dp_dr[2] / (P_[2] * P_[2]));
  A(0,1) = fx * (dp_dr[3] / P_[2] - P_[0] * dp_dr[5] / (P_[2] * P_[2]));
  A(0,2) = fx * (dp_dr[6] / P_[2] - P_[0] * dp_dr[8] / (P_[2] * P_[2]));
  A(1,0) = fy * (dp_dr[1] / P_[2] - P_[1] * dp_dr[2] / (P_[2] * P_[2]));
  A(1,1) = fy * (dp_dr[4] / P_[2] - P_[1] * dp_dr[5] / (P_[2] * P_[2]));
  A(1,2) = fy * (dp_dr[7] / P_[2] - P_[1] * dp_dr[8] / (P_[2] * P_[2]));
  A(0,3) = fx / P_[2];
  A(0,4) = 0;
  A(0,5) = - fx * P_[0] / (P_[2] * P_[2]);
  A(1,3) = 0;
  A(1,4) = fy / P_[2];
  A(1,5) = - fy * P_[1] / (P_[2] * P_[2]);	

  // corner 1
  P << a[1].x, a[1].y, a[1].z;
  P_ = R * P + t0;
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      dp_dr[j*3+i] = dr_drmat.at<float>(j,i*3+0) * P[0] + dr_drmat.at<float>(j,i*3+1) * P[1] + dr_drmat.at<float>(j,i*3+2) * P[2];
    }
  }
  A(2,0) = fx * (dp_dr[0] / P_[2] - P_[0] * dp_dr[2] / (P_[2] * P_[2]));
  A(2,1) = fx * (dp_dr[3] / P_[2] - P_[0] * dp_dr[5] / (P_[2] * P_[2]));
  A(2,2) = fx * (dp_dr[6] / P_[2] - P_[0] * dp_dr[8] / (P_[2] * P_[2]));
  A(3,0) = fy * (dp_dr[1] / P_[2] - P_[1] * dp_dr[2] / (P_[2] * P_[2]));
  A(3,1) = fy * (dp_dr[4] / P_[2] - P_[1] * dp_dr[5] / (P_[2] * P_[2]));
  A(3,2) = fy * (dp_dr[7] / P_[2] - P_[1] * dp_dr[8] / (P_[2] * P_[2]));
  A(2,3) = fx / P_[2];
  A(2,4) = 0;
  A(2,5) = - fx * P_[0] / (P_[2] * P_[2]);
  A(3,3) = 0;
  A(3,4) = fy / P_[2];
  A(3,5) = - fy * P_[1] / (P_[2] * P_[2]);

  // corner 2
  P << a[2].x, a[2].y, a[2].z;
  P_ = R * P + t0;
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      dp_dr[j*3+i] = dr_drmat.at<float>(j,i*3+0) * P[0] + dr_drmat.at<float>(j,i*3+1) * P[1] + dr_drmat.at<float>(j,i*3+2) * P[2];
    }
  }
  A(4,0) = fx * (dp_dr[0] / P_[2] - P_[0] * dp_dr[2] / (P_[2] * P_[2]));
  A(4,1) = fx * (dp_dr[3] / P_[2] - P_[0] * dp_dr[5] / (P_[2] * P_[2]));
  A(4,2) = fx * (dp_dr[6] / P_[2] - P_[0] * dp_dr[8] / (P_[2] * P_[2]));
  A(5,0) = fy * (dp_dr[1] / P_[2] - P_[1] * dp_dr[2] / (P_[2] * P_[2]));
  A(5,1) = fy * (dp_dr[4] / P_[2] - P_[1] * dp_dr[5] / (P_[2] * P_[2]));
  A(5,2) = fy * (dp_dr[7] / P_[2] - P_[1] * dp_dr[8] / (P_[2] * P_[2]));
  A(4,3) = fx / P_[2];
  A(4,4) = 0;
  A(4,5) = - fx * P_[0] / (P_[2] * P_[2]);
  A(5,3) = 0;
  A(5,4) = fy / P_[2];
  A(5,5) = - fy * P_[1] / (P_[2] * P_[2]);

  // corner 3
  P << a[3].x, a[3].y, a[3].z;
  P_ = R * P + t0;
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      dp_dr[j*3+i] = dr_drmat.at<float>(j,i*3+0) * P[0] + dr_drmat.at<float>(j,i*3+1) * P[1] + dr_drmat.at<float>(j,i*3+2) * P[2];
    }
  }
  A(6,0) = fx * (dp_dr[0] / P_[2] - P_[0] * dp_dr[2] / (P_[2] * P_[2]));
  A(6,1) = fx * (dp_dr[3] / P_[2] - P_[0] * dp_dr[5] / (P_[2] * P_[2]));
  A(6,2) = fx * (dp_dr[6] / P_[2] - P_[0] * dp_dr[8] / (P_[2] * P_[2]));
  A(7,0) = fy * (dp_dr[1] / P_[2] - P_[1] * dp_dr[2] / (P_[2] * P_[2]));
  A(7,1) = fy * (dp_dr[4] / P_[2] - P_[1] * dp_dr[5] / (P_[2] * P_[2]));
  A(7,2) = fy * (dp_dr[7] / P_[2] - P_[1] * dp_dr[8] / (P_[2] * P_[2]));
  A(6,3) = fx / P_[2];
  A(6,4) = 0;
  A(6,5) = - fx * P_[0] / (P_[2] * P_[2]);
  A(7,3) = 0;
  A(7,4) = fy / P_[2];
  A(7,5) = - fy * P_[1] / (P_[2] * P_[2]);

  /* estimate error */
  // Note : If A^T*A is singular, the previous estimated error is used.
  
  if (abs((A.transpose()*A).determinant()) > 0.001) {
    // calculate psuedo-inverse matrix
    Matrix<float, 6, 8> A_inv;
    A_inv = (A.transpose() * A).inverse() * A.transpose();

    //////////////////////////////////////////////////////////////////////
    // translational part
    
    Vector3f err_t(0, 0, 0);
    for (int i = 3; i < 6; i++){
      for (int j = 0; j < 8; j++) {
        err_t(i-3) += A_inv(i,j) * A_inv(i, j);
      }
      err_t(i-3) = float(sqrt(err_t(i-3))); 
    }

    err_t = -R.inverse() * err_t;
    err_t(0) = abs(err_t(0));
    err_t(1) = abs(err_t(1));
    err_t(2) = abs(err_t(2));
    //////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////
    // rotational part

    Vector3f dr_(0, 0, 0);
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 8; j++) {
        dr_(i) += A_inv(i,j) * A_inv(i, j);
      }
      dr_(i) = float(sqrt(dr_(i)));
    }	

    Matrix3f dR;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        dR(i,j) = dr_drmat.at<float>(i,j*3+0)*dr_drmat.at<float>(i,j*3+0)*dr_(0)*dr_(0);
        dR(i,j) += dr_drmat.at<float>(i,j*3+1)*dr_drmat.at<float>(i,j*3+1)*dr_(1)*dr_(1);
        dR(i,j) += dr_drmat.at<float>(i,j*3+2)*dr_drmat.at<float>(i,j*3+2)*dr_(2)*dr_(2);
        dR(i,j) = float(sqrt(dR(i,j)));
      }
    }
    
    Matrix3f dR_inv = -R.inverse() * dR * R.inverse();
    Vector3f err_r = dR_inv * t0;
    //////////////////////////////////////////////////////////////////////

    for (int i = 0; i < 3; i++) {
      // summing up
      err_[i] = err_t[i]*err_t[i] + err_r[i]*err_r[i];
    }
  }
}

void aruco_wrapper::geterr(vector<Marker> v_m, Mat Rvec, Mat Tvec, float err[3]) 
{
  float dst[3] = {0, 0, 0};
  
  for (int i = 0; i < v_m.size(); i++) {
    float temp[3];
    // assign previous value
    for (int j = 0; j < 3; j++) {
      temp[j] = err[j];
    }
    geterr_(v_m[i], Rvec, Tvec, temp);
    for (int j = 0; j < 3; j++) {
      dst[j] += temp[j];			
    }
  }
  
  for (int i = 0; i < 3; i++) {
    err[i] = dst[i] / v_m.size();
  }
}

void aruco_wrapper::getquaternion(Mat Rvec, float quat[3]) 
{
  Mat rmat;
  Rodrigues(Rvec, rmat);

  float angle_x, angle_y, angle_z;
  angle_x = -asin(rmat.at<float>(2,1));
  angle_y = atan2(rmat.at<float>(2,0), rmat.at<float>(2,2));
  angle_z = atan2(-rmat.at<float>(1,1), rmat.at<float>(0,1));	

  if (isnan(angle_x)) angle_x = 0;
  if (isnan(angle_y)) angle_y = 0;
  if (isnan(angle_z)) angle_z = 0;

  Quaternionf temp;
  temp = AngleAxisf(angle_x, Vector3f::UnitX()) *
    AngleAxisf(angle_y, Vector3f::UnitY()) *
    AngleAxisf(angle_z, Vector3f::UnitZ());
  
  quat[0] = temp.x();
  quat[1] = temp.y();
  quat[2] = temp.z();
}

bool aruco_wrapper::checkmarker(vector<Marker> v_m)
{
  if (!v_m.size()) return false;
  
  vector<int> ids, MM_ids;
  MM.getIdList(MM_ids);
  for (int i = 0; i < v_m.size(); i++) {
    ids.push_back(v_m[i].id);
  }
  
  vector<int>::iterator it;

  for (int i = 0; i < ids.size(); i++) {
    it = find(MM_ids.begin(), MM_ids.end(), ids[i]);
    if (it == MM_ids.end()) {
      return false;
    }
  }

  return true;
}

bool aruco_wrapper::checkpos(float position[3], float dt)
{
  if (position[2]>0||position[2]<-50||isnan(position[2])||isinf(position[2])||dt > 1000000) {
    MarkerMapPoseTracker temp;
    temp.setParams(CP, MM);		
    MMPT = temp;
    return false;
  }	
  return true;
}

// =============================================================================

// =============================================================================
// Public functions:

aruco_wrapper::aruco_wrapper(string pathforCP, string pathforMM)
{
  SetCameraParameters(pathforCP);
  SetMarkerMap(pathforMM);
  SetMarkerMapPoseTracker();
  timerclear(&tv);
  packet = {0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};
}

void aruco_wrapper::SetCameraParameters(string filepath)
{
  CP.readFromXMLFile(filepath.c_str());
}

void aruco_wrapper::SetMarkerMap(string filepath)
{
  MM.readFromFile(filepath.c_str());
}

void aruco_wrapper::SetMarkerMapPoseTracker()
{
  MMPT.setParams(CP, MM);
}

bool aruco_wrapper::MarkerUpdate(Mat img)
{
  gettimeofday(&tv, NULL);
  packet.timestamp = (tv.tv_sec % 1000) * 1000000 + tv.tv_usec;
  
  vector<Marker> v_m;
  MarkerDetector detector;
  detector.detect(img, v_m);

  bool judge = checkmarker(v_m);

  if (judge) {
    static uint32_t t_m = packet.timestamp;
    uint32_t dt = packet.timestamp - t_m; 
    t_m = packet.timestamp;

    MMPT.estimatePose(v_m);
    Mat Rvec = MMPT.getRvec();
    Mat Tvec = MMPT.getTvec();
    getpos(Rvec, Tvec, packet.position);
    
    if (checkpos(packet.position, dt)) {
      geterr(v_m, Rvec, Tvec, packet.r_var);
      getquaternion(Rvec, packet.quaternion);
    } else {
      judge = false;
    }   
  }

  packet.status = judge;

  return judge;
}

void aruco_wrapper::Disp()
{
  cout << "******************************************" << endl;
  cout << "Timestamp: " << packet.timestamp << endl;
  cout << "Position: " << packet.position[0] << "\t" << packet.position[1] << "\t" << packet.position[2] << endl;
  cout << "Variance: " << packet.r_var[0] << "\t" << packet.r_var[1] << "\t" << packet.r_var[2] << endl;
  cout << "Quaternion: " << sqrt(1-packet.quaternion[0]*packet.quaternion[0]-packet.quaternion[1]*packet.quaternion[1]-packet.quaternion[2]*packet.quaternion[2]) << "\t";
  cout << packet.quaternion[0] << "\t" << packet.quaternion[1] << "\t" << packet.quaternion[2] << endl;
}

void aruco_wrapper::Logging()
{
  fout << packet.timestamp << ",";
  fout << packet.position[0] << "," << packet.position[1] << "," << packet.position[2] << ",";
  fout << packet.quaternion[0] << "," << packet.quaternion[1] << "," << packet.quaternion[2] << ",";
  fout << packet.r_var[0] << "," << packet.r_var[1] << "," << packet.r_var[2] << ",";
  fout << int(packet.status) << "," << endl;
}

struct Packet* aruco_wrapper::Packet()
{
  return &packet;
}

// =============================================================================