#ifndef __ADCTRL_H__
#define __ADCTRL_H__

#include "json.hpp"
#include <eigen3/Eigen/Core>

class Biquad
{
private:
  // H(z) = (b0 + b1 z^(-1) + b2 z^(-2))/(a0 + a1 z^(-1) + a2 z^(-2))
  float b0, b1, b2, a0, a1, a2;
  float s1, s2;
public:
  Biquad(std::vector<float> sos);
  float update(float x);
};

class DigitalFilter
{
private:
  std::vector<Biquad> bq;
  float g; // static gain
  float n; // number of biquad cascades
public:
  DigitalFilter(std::vector<std::vector<float>> sos_vector, float g_);
  float update(float x);
};

class L1
{
private:
  // Constants
  int dim, dim_m, dim_um; // dimensions
  // Reference model
  float Ts;
  Eigen::MatrixXf Am;
  Eigen::MatrixXf Bm;
  Eigen::MatrixXf Bum;
  Eigen::MatrixXf B;
  Eigen::MatrixXf C;
  Eigen::MatrixXf Kg;
  // Adaptive law
  Eigen::MatrixXf eAmTs;
  Eigen::MatrixXf InvPhi;
  Eigen::VectorXf prediction_vector;
  Eigen::VectorXf h;
  Eigen::VectorXf sigma;
  // Control law
  std::vector<DigitalFilter> control_filters;
  float control_input;
public:
  L1(float Ts_, Eigen::MatrixXf Am_, Eigen::MatrixXf Bm_,
    Eigen::MatrixXf Bum_, Eigen::MatrixXf C_,
    std::vector<DigitalFilter> filters_);
  void show();
  float update(Eigen::VectorXf observation_vector, float reference_signal);
};


// =============================================================================
// Public functions:
L1 ReadL1Params(std::string filepath);

/*
//float AdaptiveThrustControl(float* thrust_states, float nominal_z_command);
*/

#endif //__ADCTRL_H__
