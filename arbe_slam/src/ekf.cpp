#include "ekf.h"


ExtendKalmanFilter::ExtendKalmanFilter()
{
  previous_timestamp = 0;
  dt = 0.07;

  x_est_last = VectorXd(4);
  x_est_last << 0, 0, 0, 0;
  x_pre = VectorXd(4);
  x_pre << 0, 0, 0, 0;
  x_meas = VectorXd(4);
  x_meas << 0, 0, 0, 0;
  x_est = VectorXd(4);
  x_est << 0, 0, 0, 0;

  F = MatrixXd::Zero(4, 4);
  Q = MatrixXd::Zero(4, 4);
  noise_ax = 50;
  noise_ay = 5;

  P_pre = MatrixXd(4, 4);
  P_est = MatrixXd(4, 4);
  init();


  z_est_last = VectorXd(3);
  z_est_last << 0, 0, 0;
  z_pre = VectorXd(3);
  z_pre << 0, 0, 0;
  z_meas = VectorXd(3);
  z_meas << 0, 0, 0;
  z_est = VectorXd(3);
  z_est << 0, 0, 0;

  HJ = MatrixXd::Zero(3, 4);
  R = MatrixXd(3, 3);
  R << 1, 0, 0,
    0, 0.0035, 0,
    0, 0, 0.2;

  I = MatrixXd::Identity(4, 4);

  // cout << "creat ekf object " << endl;
}

ExtendKalmanFilter::~ExtendKalmanFilter()
{
  // cout << "delet ekf object" << endl;
}

void ExtendKalmanFilter::init()
{
  P_pre << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 10, 0,
    0, 0, 0, 1;
  P_est = P_pre;
  is_initialized = false;
}

VectorXd ExtendKalmanFilter::feed(const VectorXd& z, double timestamp)
{
  z_meas = z;
  x_meas = trans_polar_to_cart(z_meas);

  if (!is_initialized)
  {
    x_est = x_meas;
    z_est = trans_cart_to_polar(x_est);

    is_initialized = true;
    // cout << "init ekf" << endl;
  }

  else
  {
    dt = timestamp - previous_timestamp;
    update();
  }

  print();

  previous_timestamp = timestamp;
  predict();

  return x_est;
}

VectorXd ExtendKalmanFilter::get_predict()
{
  return x_pre;
}

void ExtendKalmanFilter::calculate_F_Q()
{
  F << 1, 0, dt, 0,
    0, 1, 0, dt,
    0, 0, 1, 0,
    0, 0, 0, 1;

  double noise_ax_2 = pow(noise_ax, 2);
  double noise_ay_2 = pow(noise_ay, 2);
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt / 2;
  double dt_4 = dt_3 * dt / 2;

  Q << dt_4 * noise_ax_2, 0, dt_3* noise_ax_2, 0,
    0, dt_4* noise_ay_2, 0, dt_3* noise_ay_2,
    dt_3* noise_ax_2, 0, dt_2* noise_ax_2, 0,
    0, dt_3* noise_ay_2, 0, dt_2* noise_ay_2;
}

void ExtendKalmanFilter::predict()
{
  x_est_last = x_est;
  z_est_last = z_est;

  calculate_F_Q();
  x_pre = F * x_est;

  z_pre = trans_cart_to_polar(x_pre);

  MatrixXd Ft = F.transpose();
  P_pre = F * P_est * Ft + Q;
}

void ExtendKalmanFilter::calculate_jacobian()
{
  double px = x_pre(0);
  double py = x_pre(1);
  double vx = x_pre(2);
  double vy = x_pre(3);


  double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = (c1 * c2);

  if (fabs(c1) < 0.0001) {
    // cout << "ERROR - Division by Zero" << endl;
    return;
  }

  HJ << (px / c2), (py / c2), 0, 0,
    (px / c1), -(py / c1), 0, 0,
    py* (py * vx - px * vy) / c3, px* (px * vy - py * vx) / c3, px / c2, py / c2;
}

void ExtendKalmanFilter::update()
{

  VectorXd y = z_meas - z_pre;

  while (y(1) > M_PI) y(1) -= 2 * M_PI;
  while (y(1) < -M_PI) y(1) += 2 * M_PI;

  calculate_jacobian();
  MatrixXd HJt = HJ.transpose();
  MatrixXd S = HJ * P_pre * HJt + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_pre * HJt * Si;

  x_est = x_pre + (K * y);
  z_est = trans_cart_to_polar(x_est);

  P_est = (I - K * HJ) * P_pre;
}

VectorXd ExtendKalmanFilter::trans_polar_to_cart(VectorXd& polar)
{
  VectorXd cart = VectorXd(4);
  cart(0) = polar(0) * sin(polar(1));
  cart(1) = polar(0) * cos(polar(1));
  cart(2) = polar(2) * sin(polar(1));
  cart(3) = polar(2) * cos(polar(1));
  return cart;
}

VectorXd ExtendKalmanFilter::trans_cart_to_polar(VectorXd& cart)
{
  VectorXd polar = VectorXd(3);
  double px = cart(0);
  double py = cart(1);
  double vx = cart(2);
  double vy = cart(3);

  double range = sqrt(px * px + py * py);
  double azimuth = atan2(px, py);
  double doppler = (px * vx + py * vy) / range;

  polar << range, azimuth, doppler;
  return polar;
}

void ExtendKalmanFilter::print()
{
  cout << "cartesian" << endl
    << "last  : " << x_est_last(0) << " " << x_est_last(1) << " " << x_est_last(2) << " " << x_est_last(3) << endl
    << "predict : " << x_pre(0) << " " << x_pre(1) << " " << x_pre(2) << " " << x_pre(3) << endl
    << "measure : " << x_meas(0) << " " << x_meas(1) << " " << x_meas(2) << " " << x_meas(3) << endl
    << "estimate : " << x_est(0) << " " << x_est(1) << " " << x_est(2) << " " << x_est(3) << endl
    << "polar" << endl
    << "last  : " << z_est_last(0) << " " << z_est_last(1) << " " << z_est_last(2) << endl
    << "predict  : " << z_pre(0) << " " << z_pre(1) << " " << z_pre(2) << endl
    << "measure  : " << z_meas(0) << " " << z_meas(1) << " " << z_meas(2) << endl
    << "estimate  : " << z_est(0) << " " << z_est(1) << " " << z_est(2) << endl;

  cout << "P_pre : " << endl << P_pre << endl;
  cout << "P_est : " << endl << P_est << endl;
}


