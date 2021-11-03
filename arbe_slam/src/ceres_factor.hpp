#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <math.h>

struct RadarDopplerFactor
{
  RadarDopplerFactor(double doppler_, double azimuth_, double elevation_)
    : doppler(doppler_), azimuth(azimuth_), elevation(elevation_) {}

  template <typename T>
  bool operator()(const T* motion_estimate, T* residual) const
  {
    residual[0] = (motion_estimate[0] * cos(elevation) * cos(azimuth)
      + (motion_estimate[1]) * sin(elevation) * sin(azimuth)) - doppler;

    return true;
  }

  static ceres::CostFunction* Create(const double doppler_, const double azimuth_, const double elevation_)
  {
    return (new ceres::AutoDiffCostFunction<
      RadarDopplerFactor, 1, 2>(
        new RadarDopplerFactor(doppler_, azimuth_, elevation_)));
  }

  double doppler, azimuth, elevation;
};