#include "sampler.h"

#include <iostream>

namespace CMU462 {

// Uniform Sampler2D Implementation //

Vector2D UniformGridSampler2D::get_sample() const {
  // Implement uniform 2D grid sampler
  double X = (double)(std::rand()) / RAND_MAX;
  double Y = (double)(std::rand()) / RAND_MAX;
  return Vector2D(X, Y);
}

// Uniform Hemisphere Sampler3D Implementation //

Vector3D UniformHemisphereSampler3D::get_sample() const {
  double Xi1 = (double)(std::rand()) / RAND_MAX;
  double Xi2 = (double)(std::rand()) / RAND_MAX;

  double theta = acos(Xi1);
  double phi = 2.0 * PI * Xi2;

  double xs = sinf(theta) * cosf(phi);
  double ys = sinf(theta) * sinf(phi);
  double zs = cosf(theta);

  return Vector3D(xs, ys, zs);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample() const {
  float f;
  return get_sample(&f);
}

Vector3D CosineWeightedHemisphereSampler3D::get_sample(float *pdf) const {
  double d1 = randDouble();
  double d2 = randDouble();

  double phi = 2 * PI * d1;
  double z = sqrt(d2);
  double theta = acos(z);
  *pdf = (float) z / PI;
  Vector3D dir =  Vector3D(
          cos(phi) * sin(theta),
          sin(phi) * sin(theta),
          z
  );
  return dir;
}

}  // namespace CMU462
