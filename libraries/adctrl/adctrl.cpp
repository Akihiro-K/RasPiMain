// Adaptive control

#include "adctrl.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

// =============================================================================
// Private function declarations:
Eigen::MatrixXf jsonToMat(nlohmann::json j, std::string name);
void LimitAbsoluteValue(float * f, float limit_value);

// =============================================================================
// Digital filter:

Biquad::Biquad(std::vector<float> sos)
{
  b0 = sos[0];
  b1 = sos[1];
  b2 = sos[2];
  a0 = sos[3];
  a1 = sos[4];
  a2 = sos[5];
  s1 = 0;
  s2 = 0;
}

float Biquad::update(float x){
  float y;
  y = b0 * x + s1;
  s1 = b1 * x + s2 - a1 * y;
  s2 = b2 * x - a2 * y;
  return y;
}

DigitalFilter::DigitalFilter(std::vector<std::vector<float>> sos_vector, float g_)
{
  int n = sos_vector.size();
  for(int i = 0; i < n; i++){
    bq.push_back(Biquad(sos_vector[i]));
  }
  g = g_;
}

float DigitalFilter::update(float x)
{
  float y = x;
  for(int i = 0; i < n; i++){
    y = bq[i].update(y);
  }
  y *= g;
  return y;
}

// =============================================================================
// L1:

L1::L1(float Ts_, Eigen::MatrixXf Am_, Eigen::MatrixXf Bm_,
  Eigen::MatrixXf Bum_, Eigen::MatrixXf C_, std::vector<DigitalFilter> control_filters_)
{
  Ts = Ts_;
  Am = Am_;
  Bm = Bm_;
  C = C_;
  Bum = Bum_;
  Kg = -1.0 * (C*Am.inverse()*Bm).inverse();
  control_filters = control_filters_;

  dim_m = Bm.cols(); // number of matched uncertainties
  dim_um = Bum.cols(); // number of unmatched uncertainties
  dim = dim_m + dim_um; // total dimension

  B = Eigen::MatrixXf(dim,dim);
  B << Bm, Bum;

  // e^(Am*Ts) = I + (Am*Ts) + (Am*Ts)^2 / 2 + (Am*Ts)^3 / 6 + ...
  Eigen::MatrixXf eAmTs = Eigen::MatrixXf::Identity(dim,dim);
  eAmTs += (Ts) * Am;
  eAmTs += (Ts*Ts/2.0) * (Am * Am);
  eAmTs += (Ts*Ts*Ts/6.0) * (Am * Am * Am);

  // InvPhi = ( inv(Am)*(e^(Am*Ts) - eye(dim_Am))*B )^(-1)
  Eigen::MatrixXf Phi = (Am.inverse()*eAmTs - Eigen::MatrixXf::Identity(dim,dim))*B;
  Eigen::MatrixXf InvPhi = Phi.inverse();

  prediction_vector = Eigen::VectorXf::Zero(dim);
  h = Eigen::VectorXf::Zero(dim);
  sigma = Eigen::VectorXf::Zero(dim);
  control_input = 0.0;
}

void L1::show(){
  std::cout << "Ts:" << Ts << std::endl;
  std::cout << "Am:" << Am << std::endl;
  std::cout << "Bm:" << Bm << std::endl;
  std::cout << "Bum:" << Bum << std::endl;
  std::cout << "Kg:" << Kg << std::endl;
  std::cout << "number of control_filters:" << control_filters.size() << std::endl;
  std::cout << "dim_m:" << dim_m << std::endl;
  std::cout << "dim_um:" << dim_um << std::endl;
  std::cout << "dim:" << dim << std::endl;
  std::cout << "B:" << B << std::endl;
  std::cout << "h:" << h << std::endl;
}

float L1::update(Eigen::VectorXf observation_vector, float reference_signal)
{
  //------ Adaptive law ------
  Eigen::VectorXf x_tilde = prediction_vector - observation_vector;
  h -= x_tilde; // update h
  sigma = InvPhi*h - InvPhi*eAmTs*x_tilde; // update adaptive parameters

  //------ Control law ------
  float control_input = 0.0;
  // control input for matched uncertainties
  Eigen::VectorXf Kgr = Kg*reference_signal;
  for(int i = 0; i < dim_m; i++){
    control_input -= control_filters[i].update(sigma[i]-Kgr[i]);
  }
  // control input for unmatched uncertainties
  for(int i = dim_m; i < dim; i++){
    control_input -= control_filters[i].update(sigma[i]);
  }

  //------ State predictor ------
  // xdot = Am*x + Bm*u + B*sigma
  Eigen::VectorXf prediction_derivative;
  prediction_derivative = Am*prediction_vector + Bm*control_input + B*sigma;
  prediction_vector += Ts*prediction_derivative;

  return control_input;
}

/*
float AdaptiveThrustControl(float* thrust_states, float nominal_z_command)
{
  // thrust_states: wdot, w, z
  float adaptive_thrust_command = AdaptiveControl(&l1z, thrust_states, nominal_z_command);
  return adaptive_thrust_command;
}

*/


// =============================================================================
// Public functions:
L1 ReadL1Params(std::string filepath){
  std::ifstream ifs(filepath);
  nlohmann::json j;
  ifs >> j;

  float Ts = j["Ts"];
  Eigen::MatrixXf Am = jsonToMat(j,"Am");
  Eigen::MatrixXf Bm = jsonToMat(j,"Bm");
  Eigen::MatrixXf Bum = jsonToMat(j,"Bum");
  Eigen::MatrixXf C = jsonToMat(j,"C");

  std::vector<DigitalFilter> control_filters;
  for(int i = 0; i < Bm.size(); i++){
    // read sos_vector
    std::vector<std::vector<float>> sos_vector;
    std::vector<float> sos(6);
    int nsos = j["control_filters"][i]["sos"].size(); // number of sos
    for(int k = 0; k < nsos; k++){
      for(int l = 0; l < 6; l++){
        sos.push_back(j["control_filters"][i]["sos"][k][l]);
      }
      sos_vector.push_back(sos);
    }
    // read gain
    float g = j["control_filters"][i]["gain"];
    // append filter
    control_filters.push_back(DigitalFilter(sos_vector,g));
  }

  return L1(Ts,Am,Bm,Bum,C,control_filters);
}

// =============================================================================
// Private functions:
Eigen::MatrixXf jsonToMat(nlohmann::json j, std::string name){
  int rows = j[name].size();
  int cols = j[name][0].size();
  Eigen::MatrixXf mat(rows,cols);
  for(int l = 0; l < rows; l++){
    for(int m = 0; m < cols; m++){
      mat(l,m) = j[name][l][m];
    }
  }
  return mat;
}

void LimitAbsoluteValue(float * f, float limit_value)
{
  //if(limit_value < 0) return;

  if(*f > limit_value){
    *f = limit_value;
  }else if(*f < -limit_value){
    *f = -limit_value;
  }
}
