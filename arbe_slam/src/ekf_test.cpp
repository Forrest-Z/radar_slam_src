#include "ekf.h"


void plot(cv::Mat& result, double px, double py, double vx, double vy, double dvx, double dvy, int tx, int ty)
{
  srand(time(0));
  double dt = 0.1;

  double range_noise_scale = 0.5;
  double azimuth_noise_scale = 0.02;
  double doppler_noise_scale = 0.1;

  ExtendKalmanFilter ekf;
  // ekf.init();

  cv::Scalar color_real = cv::Scalar(0, 0, 255);
  cv::Scalar color_meas = cv::Scalar(0, 255, 0);
  cv::Scalar color_ekf = cv::Scalar(255, 0, 0);

  VectorXd x_meas = VectorXd(4);
  VectorXd x_ekf = VectorXd(4);

  int radius_real = 2;
  int radius_meas = 6;
  int radius_ekf = 4;

  double variance_meas = 0.0;
  double variance_ekf = 0.0;
  for (int i = 0;i < 100;i++)
  {
    px += vx * dt;
    py += vy * dt;

    double noise = rand() % 10 * 0.1;

    double range = sqrt(px * px + py * py) + noise * range_noise_scale;
    range = double(int(range / 0.5 + 0.5)) * 0.5;
    double azimuth = atan2(px, py) + noise * azimuth_noise_scale;
    azimuth = double(int(azimuth / 0.0174 + 0.5)) * 0.0174;
    double doppler = (px * vx + py * vy) / range + noise * doppler_noise_scale;
    doppler = double(int(doppler / 0.1 + 0.5)) * 0.1;

    VectorXd z = VectorXd(3);
    z << range, azimuth, doppler;
    x_ekf = ekf.feed(z, i * dt);

    vx += dvx;
    vy += dvy;

    x_meas(0) = range * sin(azimuth);
    x_meas(1) = range * cos(azimuth);
    x_meas(2) = doppler * sin(azimuth);
    x_meas(2) = doppler * cos(azimuth);

    cv::Point2d point_real = cv::Point2d((int)px * 10, (int)py * 10);
    cv::Point2d point_meas = cv::Point2d((int)x_meas(0) * 10, (int)x_meas(1) * 10);
    cv::Point2d point_ekf = cv::Point2d((int)x_ekf(0) * 10, (int)x_ekf(1) * 10);

    cv::circle(result, point_real, radius_real, color_real, 2);
    cv::circle(result, point_meas, radius_meas, color_meas, 2);
    cv::circle(result, point_ekf, radius_ekf, color_ekf, 2);

    variance_meas += (x_meas(0) - px) * (x_meas(0) - px) + (x_meas(1) - py) * (x_meas(1) - py);
    variance_ekf += (x_ekf(0) - px) * (x_ekf(0) - px) + (x_ekf(1) - py) * (x_ekf(1) - py);
  }

  string txt_meas = "measure : " + to_string(variance_meas / 100.0);
  string txt_ekf = "ekf : " + to_string(variance_ekf / 100.0);

  cv::Point2d point_txt_real = cv::Point2d(tx, ty);
  cv::putText(result, "real", point_txt_real, cv::FONT_HERSHEY_COMPLEX, 1, color_real, 2);
  cv::Point2d point_txt_meas = cv::Point2d(tx, ty + 50);
  cv::putText(result, txt_meas, point_txt_meas, cv::FONT_HERSHEY_COMPLEX, 1, color_meas, 2);
  cv::Point2d point_txt_ekf = cv::Point2d(tx, ty + 100);
  cv::putText(result, txt_ekf, point_txt_ekf, cv::FONT_HERSHEY_COMPLEX, 1, color_ekf, 2);
}

int main()
{
  cv::Mat result = cv::Mat(1200, 1200, CV_8UC3, cv::Vec3b(255, 255, 255));

  plot(result, 5, 0, 0, 14, 0.1, 0, 20, 900);
  plot(result, 5, 0, 0, 14, 0.2, 0, 600, 1050);
  plot(result, 5, 0, 0, 14, 0.3, 0, 600, 900);

  plot(result, 5, 5, 14, 14, 0, 0, 400, 500);

  plot(result, 0, 5, 14, 0, 0, 0.3, 920, 700);
  plot(result, 0, 5, 14, 0, 0, 0.2, 920, 500);
  plot(result, 0, 5, 14, 0, 0, 0.1, 920, 300);


  cv::imshow("ekf", result);
  cv::waitKey();
  cv::imwrite("/home/qinguoyu/ekf.jpg", result);

  return 0;

}
