#include <iostream>
#include "environment_light.h"

namespace CMU462 {
namespace StaticScene {

EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap), width(envMap->w), height(envMap->h) {
  if (width == 0 || height == 0) return;

  std::vector<double> p_theta_phi = std::vector<double>(width * height);
  p_theta_cdf.resize(height);
  p_theta.resize(height);
  p_phi_cond_theta.resize(width * height);

  double total = 0;
  for (auto j = 0; j < height; ++j) {
    for (auto i = 0; i < width; ++i) {
      double theta = (double)j / height * PI;
      double L = envMap->data[i + j*width].illum() * sin(theta);
      p_theta_phi[i + j*width] = L;
      total +=L;
    }
  }

  // joint p
  for (auto j = 0; j < height; ++j) {
    for (auto i = 0; i < width; ++i) {
      p_theta_phi[i + j*width] /= total;
    }
  }

  // marginal p
  for (auto j = 0; j < height; ++j) {
    for (auto i = 0; i < width; ++i) {
      p_theta[j] += p_theta_phi[j*width + i];
    }
    p_theta_cdf[j] += p_theta[j];
    if (j > 0) {
      p_theta_cdf[j] += p_theta_cdf[j - 1];
    }
  }

  // cond p cdf
  for (auto j = 0; j < height; ++j) {
    p_phi_cond_theta[j*width] = p_theta_phi[j*width]/p_theta[j];
    for (auto i = 1; i < width; ++i) {
      p_phi_cond_theta[i + j*width] = p_theta_phi[i + j*width]/p_theta[j] + p_phi_cond_theta[i-1 + j*width];
    }
  }

}

Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                    float* distToLight, float* pdf) const {
  double x1 = randDouble(),
         x2 = randDouble();

  double theta, phi;
  size_t x, y;
  size_t start = 0, end = height, mid;
  while (p_theta_cdf[start] < x1 && start < end - 1) {
    mid = (start + end) / 2;
    if (p_theta_cdf[mid] < x1) {
      start = mid;
    } else {
      end = mid;
    }
  }
  y = start;

  start = 0;
  end = width;
  while (p_phi_cond_theta[start + width * y] < x2 && start < end - 1) {
    mid = (start + end) / 2;
    if (p_phi_cond_theta[mid + width * y] < x2) {
      start = mid;
    } else {
      end = mid;
    }
  }
  x = start;

  *distToLight = INF_D;
  double cond_p = p_phi_cond_theta[x + y * width];
  if (x > 0) {
    cond_p -= p_phi_cond_theta[x - 1 + y * width];
  }
  theta = (double)y / height * PI;
  phi = (double)x / width * 2 * PI;
  *pdf = cond_p * p_theta[y] * width * height * sin(theta)/(4*PI);
  *wi = Vector3D(cos(theta), sin(theta) * cos(phi), sin(theta)*sin(phi));

  return getSample(theta, phi);
}

Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
  double theta, phi;
  transformCoordinate(r.d.unit(), theta, phi);
  return getSample(theta, phi);
}

Spectrum EnvironmentLight::getSample(double theta, double phi) const {
  size_t x_l, y_l, x_h, y_h;
  double sx = phi/(2*PI) * width,
         sy = (theta/(PI)) * height;
  x_l = (size_t) sx;
  y_l = (size_t) sy;

  x_h = (x_l + 1) % width;
  y_h = (y_l + 1);

  Spectrum s_ll, s_lr, s_ul, s_ur;
  s_ll = envMap->data[x_l + width*y_l];
  s_lr = envMap->data[x_h + width*y_l];
  // sphere wrapping
  if (y_h == height) {
    s_ul = envMap->data[x_l + width/2 + width*y_l];
    s_ur = envMap->data[x_h + width/2 + width*y_l];
  } else {
    s_ul = envMap->data[x_l + width*y_h];
    s_ur = envMap->data[x_h + width*y_h];
  }
  Spectrum lower = s_ll*(1 - sx + x_l) + s_lr * (sx - x_l),
           upper = s_ul*(1 - sx + x_l) + s_ur * (sx - x_l);
  return lower*(1 - sy + y_l) + upper*(sy - y_l);

}


void EnvironmentLight::transformCoordinate(const Vector3D& w, double& theta, double& phi) {
  theta = acos(w.y);
  double sin_theta = sin(abs(theta));
  double cos_phi = (sin_theta == 0.0 ? 0 : w.x/sin_theta);
  phi = acos(cos_phi);
  if (w.z < 0) phi = fmod(-phi + 2*PI, 2*PI);
  phi = fmod(phi + 1.5 * PI , 2*PI); // rotation by PI/2 to match reference
}

}  // namespace StaticScene
}  // namespace CMU462
