#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::min;
using std::max;
using std::swap;

namespace CMU462 {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
    h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
    h.y = 1.0;
  else
    h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return albedo * (1.0 / PI);
}

Spectrum DiffuseBSDF::get_albedo(){
  return albedo;
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
//  *wi = uniform_sampler.get_sample();
//  *pdf = (float)1 / (2 * PI);
  *wi = sampler.get_sample(pdf);
  return f(wo, *wi);
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  Spectrum out = Spectrum();
  if (isReflection(wo, wi)) {
    out = 1.l / abs_cos_theta(wi) * reflectance;
  }
  return out;
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  reflect(wo, wi);
  *pdf = 1;
  return f(wo, *wi);
}

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi,
                                  float* pdf) {
  // Implement RefractionBSDF
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  bool is_refraction = refract(wo, wi, ior);
  if (!is_refraction) {
    // total internal reflection
    *pdf = 1;
    return transmittance * (1/abs_cos_theta(*wi));
  } else {
    double r_para, r_perp;
    double eta_i = 1.0,
           eta_t = ior;
    if (wo.z < 0) std::swap(eta_i, eta_t);
    double theta_i = abs_cos_theta(wo);
    double theta_t = abs_cos_theta(*wi);
    r_para = (eta_i * cos(theta_t) - eta_t * cos(theta_i)) / (eta_i * cos(theta_t) + eta_t * cos(theta_i));
    r_perp = (eta_t * cos(theta_t) - eta_i * cos(theta_i)) / (eta_i * cos(theta_i) + eta_t * cos(theta_t));
    double Fr = 0.5 * (r_para*r_para + r_perp*r_perp);

    double rr = randDouble();
    if (rr > Fr) {
      // refraction
      *pdf = 1 - Fr;
      return transmittance * ((eta_i*eta_i*(1-Fr)) /(abs_cos_theta(*wi) * eta_t*eta_t));
    } else {
      // reflection
      *pdf = Fr;
      reflect(wo, wi);
      return reflectance * (Fr/abs_cos_theta(*wi));
    }
  }
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = Vector3D(
          -wo.x,
          -wo.y,
          wo.z
  );
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
  double eta_i = 1.0,
         eta_t = (double)ior;
  if (wo.z < 0) std::swap(eta_i, eta_t);

  double cos_i = abs_cos_theta(wo);
  double sin_i = sqrt(1 - cos_i * cos_i);
  double sin_t = sin_i * eta_i / eta_t;
  double cos_t2 = 1 - sin_t * sin_t;
  if (cos_t2 < 0) { // total internal reflection
    reflect(wo, wi);
    return false;
  }
  double cos_t = sqrt(cos_t2);
  double cos_r = - cos_t / cos_i;
  double sin_r = - sin_t / sin_i;

  // refraction
  *wi = Vector3D(
    wo.x * sin_r,
    wo.y * sin_r,
    wo.z * cos_r
    );
  return true;
}

bool BSDF::isReflection(const Vector3D& wo, const Vector3D& wi) {
  Vector3D w;
  reflect(wo, &w);
  return abs(w.x - wi.x) < EPS_D
    && abs(w.y - wi.y) < EPS_D
    && abs(w.z - wi.z) < EPS_D;
}

bool BSDF::isRefraction(const Vector3D& wo, const Vector3D& wi, double ior) {
  Vector3D w;
  bool res = refract(wo, &w, ior);
  if (!res) return false;

  return abs(w.x - wi.x) < EPS_D
    && abs(w.y - wi.y) < EPS_D
    && abs(w.z - wi.z) < EPS_D;
}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi = sampler.get_sample(pdf);
  return Spectrum();
}

}  // namespace CMU462
