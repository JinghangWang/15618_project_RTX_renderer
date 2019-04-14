#include "sphere.h"

#include <cmath>

#include "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 {
namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {
  double a, b, c;
  Vector3D rel_o = r.o - o;
  a = dot(r.d, r.d);
  b = 2* dot(rel_o, r.d);
  c = dot(rel_o, rel_o) - r2;
  double test = b*b - 4*a*c;
  if (test < 0)
    return false;

  if (abs(test) < std::numeric_limits<double>::min()) {
    t1 = -b / (2*a);
    t2 = -INF_D;
  } else {
    t1 = (-b - sqrt(test)) / (2*a);
    t2 = (-b + sqrt(test)) / (2*a);
    assert(t1 < t2);
  }
  return true;
}

bool Sphere::intersect(const Ray& r) const {
  return intersect(r, NULL);
}

bool Sphere::intersect(const Ray& r, Intersection* isect) const {
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2, t;

  if (!test(r, t1, t2))
    return false;
  // a hit is found
  if (t1 > r.min_t && t1 < r.max_t)
    t = t1;
  else if (t2 > r.min_t && t2 < r.max_t)
    t = t2;
  else
    return false;

  r.max_t = t;
  if (isect) {
    isect->t = t;
    isect->primitive = this;
    isect->bsdf = object->get_bsdf();
    isect->n = normal(r.o + t*r.d);
  }
  return true;
}

void Sphere::draw(const Color& c) const { Misc::draw_sphere_opengl(o, r, c); }

void Sphere::drawOutline(const Color& c) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

Vector3D Sphere::getCentroid() const {
  return o;
}

}  // namespace StaticScene
}  // namespace CMU462
