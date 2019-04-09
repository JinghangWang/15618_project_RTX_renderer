#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

bool BBox::intersect(const Ray &r, double &t0, double &t1) const {
  // TODO (PathTracer):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  return false;
}

double BBox::intersect(const Ray &r) const {
  double closedt_t = INF_D;
  // x planes
  if (r.d.x != 0) {
    double t1 = (min.x - r.o.x) * r.inv_d.x;
    double t2 = (max.x - r.o.x) * r.inv_d.x;
    if (t1 > r.min_t && t1 < r.max_t && t1 < closedt_t) {
      Vector3D p1 = r.o + t1 * r.d;
      if (p1.y >= min.y && p1.y <= max.y
        && p1.z >= min.z && p1.z <= max.z)
        closedt_t = t1;
    }
    if (t2 > r.min_t && t2 < r.max_t && t2 < closedt_t) {
      Vector3D p2 = r.o + t2 * r.d;
      if (p2.y >= min.y && p2.y <= max.y
        && p2.z >= min.z && p2.z <= max.z)
        closedt_t = t2;
    }
  }

  // y planes
  if (r.d.y != 0) {
    double t1 = (min.y - r.o.y) * r.inv_d.y;
    double t2 = (max.y - r.o.y) * r.inv_d.y;
    if (t1 > r.min_t && t1 < r.max_t && t1 < closedt_t) {
      Vector3D p1 = r.o + t1 * r.d;
      if (p1.x >= min.x && p1.x <= max.x
          && p1.z >= min.z && p1.z <= max.z)
        closedt_t = t1;
    }
    if (t2 > r.min_t && t2 < r.max_t && t2 < closedt_t) {
      Vector3D p2 = r.o + t2 * r.d;
      if (p2.x >= min.x && p2.x <= max.x
          && p2.z >= min.z && p2.z <= max.z)
        closedt_t = t2;
    }
  }

  // z planes
  if (r.d.z != 0) {
    double t1 = (min.z - r.o.z) * r.inv_d.z;
    double t2 = (max.z - r.o.z) * r.inv_d.z;
    if (t1 > r.min_t && t1 < r.max_t && t1 < closedt_t) {
      Vector3D p1 = r.o + t1 * r.d;
      if (p1.y >= min.y && p1.y <= max.y
          && p1.x >= min.x && p1.x <= max.x)
        closedt_t = t1;
    }
    if (t2 > r.min_t && t2 < r.max_t && t2 < closedt_t) {
      Vector3D p2 = r.o + t2 * r.d;
      if (p2.y >= min.y && p2.y <= max.y
          && p2.x >= min.x && p2.x <= max.x)
        closedt_t = t2;
    }
  }
  return closedt_t;
}

void BBox::draw(Color c) const {
  glColor4f(c.r, c.g, c.b, c.a);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();
}

std::ostream &operator<<(std::ostream &os, const BBox &b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

}  // namespace CMU462
