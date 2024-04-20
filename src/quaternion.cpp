#include "quaternion.hpp"

#include <cmath>

// TODO: Convert this to a class

void Quaternion::normalize() {
  double mod = sqrt(
      x * x +
      y * y +
      z * z +
      w * w);

  x *= 1 / mod;
  y *= 1 / mod;
  z *= 1 / mod;
  w *= 1 / mod;
}


Quaternion::Quaternion(double roll, double pitch, double yaw) {
  // Hopefully this works to convert euler angles to a quaternion
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  w = cy * cp * cr + sy * sp * sr;
  x = cy * cp * sr - sy * sp * cr;
  y = sy * cp * sr + cy * sp * cr;
  z = sy * cp * cr - cy * sp * sr;

}