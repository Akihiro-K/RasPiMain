#include "stateestimator.h"

void PositionTimeUpdate()
{
  float dt = 1.0/128;

  MatrixXf A(6,6);
  A << 1,0,0,dt,0,0,
      0,1,0,0,dt,0,
      0,0,1,0,0,dt,
      0,0,0,1,0,0,
      0,0,0,0,1,0,
      0,0,0,0,0,1;

  MatrixXf B(6,3);
  B << 0,0,0,
      0,0,0,
      0,0,0,
      g*dt,0,0,
      0,g*dt,0,
      0,0,g*dt;

  MatrixXf Q(6,6);
  Q << 0.01,0,0,0,0,0,
        0,0.01,0,0,0,0,
        0,0,0.01,0,0,0,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1;

    if ((to_fc.navigation_status&HeadingOK)&&
    (to_fc.navigation_status&PositionOK)&&
    (to_fc.navigation_status&VelocityOK)) {
    // if heading correction is performed
    // calculate a_i from quaternion

    Vector3f a_b(from_fc.accelerometer[0],from_fc.accelerometer[1],from_fc.accelerometer[2]);
    Quaternionf q_c(quat[0],quat[1],quat[2],quat[3]);
    Matrix3f DCM = q_c.toRotationMatrix(); // body to inertial
    u = DCM * a_b + Vector3f::UnitZ(); // remove acceleration due to gravity
  } else {
    for (int i = 0; i < 3; i++) {
      u(i) = 0;
    }
  }

  // if heading, velocity and position is not ok
  // position is unchanged

  x = A * x + B * u;
  P_pos = A * P_pos * A.transpose() + Q;

  for (int i = 0; i < 3; i++) {
    to_fc.position[i] = x(i);
    to_fc.velocity[i] = x(i+3);
  }
}

void PositionMeasurementUpdateWithMarker()
{
  static uint8_t init_marker_flag = 0;

  if (marker_flag) {
    // when marker is detected for the first time
    // position is initialized to raw reading from marker
    if (!init_marker_flag) {
      for (int i = 0; i < 3; i++) {
        x(i) = from_marker.position[i];
      }
    }
    init_marker_flag = 1;

    Vector3f z(from_marker.position[0], from_marker.position[1],from_marker.position[2]);
    MatrixXf H(3,6);
    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;

    Matrix3f R;
    R << from_marker.r_var[0], 0, 0,
        0, from_marker.r_var[1], 0,
        0, 0, from_marker.r_var[2];

    MatrixXf K(6,3);
    K = P_pos * H.transpose() * (H * P_pos * H.transpose() + R).inverse();

    x = x + K * (z - H * x);
    P_pos = (MatrixXf::Identity(6,6)-K * H) * P_pos;

    for (int i = 0; i < 3; i++) {
      to_fc.position[i] = x(i);
      to_fc.velocity[i] = x(i+3);
    }
  }
}

void PositionMeasurementUpdateWithGPSPos()
{
  if (gps_pos_flag) {
    Vector2f z(from_gps.position[0], from_gps.position[1]);
    MatrixXf H(2,6);
    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0;

    Matrix2f R;
    R << from_gps.r_var[0], 0,
        0, from_gps.r_var[1];

    MatrixXf K(6,2);
    K = P_pos * H.transpose() * (H * P_pos * H.transpose() + R).inverse();

    x = x + K * (z - H * x);
    P_pos = (MatrixXf::Identity(6,6)-K * H) * P_pos;

    for (int i = 0; i < 3; i++) {
      to_fc.position[i] = x(i);
      to_fc.velocity[i] = x(i+3);
    }
  }
}

void PositionMeasurementUpdateWithGPSVel()
{
  if (gps_vel_flag) {
    Vector2f z(from_gps.velocity[0], from_gps.velocity[1]);
    MatrixXf H(2,6);
    H << 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 1, 0;

    Matrix2f R;
    R << from_gps.v_var[0], 0,
        0, from_gps.v_var[1];

    MatrixXf K(6,2);
    K = P_pos * H.transpose() * (H * P_pos * H.transpose() + R).inverse();

    x = x + K * (z - H * x);
    P_pos = (MatrixXf::Identity(6,6)-K * H) * P_pos;

    for (int i = 0; i < 3; i++) {
      to_fc.position[i] = x(i);
      to_fc.velocity[i] = x(i+3);
    }
  }
}

void PositionMeasurementUpdateWithBar()
{
  VectorXf z(1);
  z << from_fc.pressure_alt;
  MatrixXf H(1,6);
  H << 0, 0, 1, 0, 0, 0;

  VectorXf R(1);
  R << 1.0;// TO DO

  MatrixXf K(6,1);
  K = P_pos * H.transpose() * (H * P_pos * H.transpose() + R).inverse();

  x = x + K * (z - H * x);
  P_pos = (MatrixXf::Identity(6,6)-K * H) * P_pos;

  for (int i = 0; i < 3; i++) {
    to_fc.position[i] = x(i);
    to_fc.velocity[i] = x(i+3);
  }
}

void AttitudeTimeUpdate()
{
  Matrix3f A;
  A << 0, from_fc.gyro[2], -from_fc.gyro[1],
        -from_fc.gyro[2], 0, from_fc.gyro[0],
        from_fc.gyro[1], -from_fc.gyro[0], 0;

  Matrix3f Q;
  Q << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;

  P_att = A * P_att * A.transpose() + Q;

  for (int i = 0; i < 4; i++) {
    quat[i] = from_fc.quaternion[i];
  }
}

void AttitudeMeasurementUpdateWithMarker()
{
  if (marker_flag) {
    Quaternionf q_marker(sqrt(1-from_marker.quaternion[0]*from_marker.quaternion[0]-from_marker.quaternion[1]*from_marker.quaternion[1]-from_marker.quaternion[2]*from_marker.quaternion[2]),
    from_marker.quaternion[0], from_marker.quaternion[1], from_marker.quaternion[2]);
    Matrix3f DCM = (q_marker.toRotationMatrix()).transpose(); // inertial to body
    Vector3f z_meas = DCM * Vector3f::UnitX();

    Quaternionf q(quat[0],quat[1],quat[2],quat[3]);
    DCM = (q.toRotationMatrix()).transpose(); // inertial to body
    Vector3f z_pred = DCM * Vector3f::UnitX();

    Vector3f dz = z_meas - z_pred;

    Matrix3f H;
    H << 0, -z_pred(2), z_pred(1),
         z_pred(2), 0, -z_pred(0),
         -z_pred(1), z_pred(0), 0;

    Matrix3f R;
    R << 0.000001, 0, 0,
         0, 0.000001, 0,
         0, 0, 0.000001;

    Matrix3f K;
    K = P_att * H.transpose() * (H * P_att * H.transpose() + R).inverse();

    P_att = (MatrixXf::Identity(3,3)-K * H) * P_att;

    Vector3f alpha = K * dz;
    MatrixXf Psi(4,3); // Conversion matrix from alpha to delta quaternion
    Psi << -0.5*quat[1], -0.5*quat[2], -0.5*quat[3],
            0.5*quat[0], -0.5*quat[3], 0.5*quat[2],
            0.5*quat[3], 0.5*quat[0], -0.5*quat[1],
            -0.5*quat[2], 0.5*quat[1], 0.5*quat[0];
    Vector4f dq = Psi * alpha;

    float norm = 0;
    for (int i = 0; i < 4; i++) {
      quat[i] += dq(i);
      norm += quat[i]*quat[i];
    }
    for (int i = 0; i < 4; i++) {
      quat[i] /= norm;
    }

    Quaternionf q_fc(from_fc.quaternion[0], from_fc.quaternion[1], from_fc.quaternion[2], from_fc.quaternion[3]);
    Quaternionf q_nc(quat[0], quat[1], quat[2], quat[3]);
    Quaternionf dq_ = q_fc.conjugate() * q_nc;

    to_fc.quat0 = dq_.w() / sqrt(dq_.w()*dq_.w()+dq_.z()*dq_.z());
    to_fc.quatz = dq_.z() / sqrt(dq_.w()*dq_.w()+dq_.z()*dq_.z());

    if (isnan(to_fc.quat0)||isnan(to_fc.quatz)) {
      to_fc.quat0 = 1;
      to_fc.quatz = 0;
    }
  }
}

void AttitudeMeasurementUpdateWithLSM()
{
  if (lsm_flag) {
    Vector3f z_meas(from_lsm.mag[0], from_lsm.mag[1], from_lsm.mag[2]);

    float initial_heading = 0.0; // TODO: create a function that defines initial heading (in radians)
    Vector3f z_pred(F*cos(I*M_PI/180), 0, F*sin(I*M_PI/180));
    Matrix3f getMi;
    getMi << cos(initial_heading-D*M_PI/180), sin(initial_heading-D*M_PI/180), 0,
              -sin(initial_heading-D*M_PI/180), cos(initial_heading-D*M_PI/180), 0,
              0, 0, 1;
    z_pred = getMi * z_pred; // magnetic vector in inertial frame

    Quaternionf q(quat[0],quat[1],quat[2],quat[3]);
    Matrix3f DCM = (q.toRotationMatrix()).transpose(); // inertial to body
    z_pred = DCM * z_pred; // magnetic vector prediction in body frame

    Vector3f dz = z_meas - z_pred;

    Matrix3f H;
    H << 0, -z_pred(2), z_pred(1),
         z_pred(2), 0, -z_pred(0),
         -z_pred(1), z_pred(0), 0;

    Matrix3f R;
    R << 0.1, 0, 0,
         0, 0.1, 0,
         0, 0, 0.1;

    Matrix3f K;
    K = P_att * H.transpose() * (H * P_att * H.transpose() + R).inverse();

    P_att = (MatrixXf::Identity(3,3)-K * H) * P_att;

    Vector3f alpha = K * dz;
    MatrixXf Psi(4,3); // Conversion matrix from alpha to delta quaternion
    Psi << -0.5*quat[1], -0.5*quat[2], -0.5*quat[3],
            0.5*quat[0], -0.5*quat[3], 0.5*quat[2],
            0.5*quat[3], 0.5*quat[0], -0.5*quat[1],
            -0.5*quat[2], 0.5*quat[1], 0.5*quat[0];
    Vector4f dq = Psi * alpha;

    float norm = 0;
    for (int i = 0; i < 4; i++) {
      quat[i] += dq(i);
      norm += quat[i]*quat[i];
    }
    for (int i = 0; i < 4; i++) {
      quat[i] /= norm;
    }

    Quaternionf q_fc(from_fc.quaternion[0], from_fc.quaternion[1], from_fc.quaternion[2], from_fc.quaternion[3]);
    Quaternionf q_nc(quat[0], quat[1], quat[2], quat[3]);
    Quaternionf dq_ = q_fc.conjugate() * q_nc;

    to_fc.quat0 = dq_.w() / sqrt(dq_.w()*dq_.w()+dq_.z()*dq_.z());
    to_fc.quatz = dq_.z() / sqrt(dq_.w()*dq_.w()+dq_.z()*dq_.z());
  }
}

void AttitudeMeasurementUpdateWithGPSVel()
{
  if (gps_vel_flag) {
    // Vector3f z_meas(from_gps.velocity[0], from_gps.velocity[1], 0);
    // z_meas.normalize();

    // Vector3f g_b_cmd(from_fc.g_b_cmd[0], from_fc.g_b_cmd[1], 0);
    // g_b_cmd.normalize();
    // Quaternionf q(quat[0],quat[1],quat[2],quat[3]);
    // Matrix3f DCM = q.toRotationMatrix(); // body to inertial
    // Vector3f z_pred = DCM * g_b_cmd; // direction of g_b_cmd is the same as that of gps_vel

    // Vector3f dz = z_meas - z_pred;

    // Matrix3f H;
    // H << 0, -z_pred(2), z_pred(1),
    //      z_pred(2), 0, -z_pred(0),
    //      -z_pred(1), z_pred(0), 0;

    // Matrix3f R;
    // R << 0.000001, 0, 0,
    //      0, 0.000001, 0,
    //      0, 0, 0.000001;

    // Matrix3f K;
    // K = P_att * H.transpose() * (H * P_att * H.transpose() + R).inverse();

    // P_att = (MatrixXf::Identity(3,3)-K * H) * P_att;

    // Vector3f alpha = K * dz;
    // MatrixXf Psi(4,3); // Conversion matrix from alpha to delta quaternion
    // Psi << -0.5*quat[1], -0.5*quat[2], -0.5*quat[3],
    //         0.5*quat[0], -0.5*quat[3], 0.5*quat[2],
    //         0.5*quat[3], 0.5*quat[0], -0.5*quat[1],
    //         -0.5*quat[2], 0.5*quat[1], 0.5*quat[0];
    // Vector4f dq = Psi * alpha;

    // float norm = 0;
    // for (int i = 0; i < 4; i++) {
    //   quat[i] += dq(i);
    //   norm += quat[i]*quat[i];
    // }
    // for (int i = 0; i < 4; i++) {
    //   quat[i] /= norm;
    // }

    // Quaternionf q_fc(from_fc.quaternion[0], from_fc.quaternion[1], from_fc.quaternion[2], from_fc.quaternion[3]);
    // Quaternionf q_nc(quat[0], quat[1], quat[2], quat[3]);
    // Quaternionf dq_ = q_fc.conjugate() * q_nc;

    // to_fc.quat0 = dq_.w() / sqrt(dq_.w()*dq_.w()+dq_.z()*dq_.z());
    // to_fc.quatz = dq_.z() / sqrt(dq_.w()*dq_.w()+dq_.z()*dq_.z());
  }
}

void ResetHeadingCorrectionQuat()
{
  if(to_fc.quat0 != 1){
    // measurement update performed in the previous timestep
    to_fc.quat0 = 1;
    to_fc.quatz = 0;
  }
}
