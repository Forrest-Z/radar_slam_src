#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <ctime>

using namespace Eigen;
using namespace std;

class ExtendKalmanFilter
{
private:
  bool is_initialized;
  double previous_timestamp;
  double dt;

  VectorXd x_est_last;
  VectorXd x_pre;
  VectorXd x_meas;
  VectorXd x_est;

  MatrixXd F;
  MatrixXd Q;

  double noise_ax;
  double noise_ay;

  MatrixXd R;

  VectorXd z_est_last;
  VectorXd z_pre;
  VectorXd z_meas;
  VectorXd z_est;

  MatrixXd HJ;

  MatrixXd P_pre;
  MatrixXd P_est;

  MatrixXd I;

public:
  ExtendKalmanFilter();
  ~ExtendKalmanFilter();
  void init();
  void calculate_F_Q();
  void predict();
  void update();
  void calculate_jacobian();
  VectorXd feed(const VectorXd& z, double timestamp);
  VectorXd get_predict();
  void print();

  VectorXd trans_polar_to_cart(VectorXd& polar);
  VectorXd trans_cart_to_polar(VectorXd& cart);
};