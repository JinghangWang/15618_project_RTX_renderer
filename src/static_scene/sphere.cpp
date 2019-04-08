#include "sphere.h"

#include <cmath>

#include "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 {
namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  
  return false;
}

bool Sphere::intersect(const Ray& r) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  return false;
}

bool Sphere::intersect(const Ray& r, Intersection* isect) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double a, b, c;
  Vector3D rel_o = r.o - o;
  a = dot(r.d, r.d);
  b = 2* dot(rel_o, r.d);
  c = dot(rel_o, rel_o) - r2;
  double test = b*b - 4*a*c;
  if (test < 0)
    return false;

  double t;
  if (abs(test) < std::numeric_limits<double>::min()) {
    t = -b / (2*a);
  } else {
    t = (-b - sqrt(test)) / (2*a);
  }
  if (t < r.min_t || t > r.max_t)
    return false;

  // a hit is found
  r.max_t = t;
  if (isect != NULL) {
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
