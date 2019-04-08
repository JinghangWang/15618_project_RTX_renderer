#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 {
namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, vector<size_t>& v) : mesh(mesh), v(v) {}
Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3)
    : mesh(mesh), v1(v1), v2(v2), v3(v3) {}

BBox Triangle::get_bbox() const {
  // TODO (PathTracer):
  // compute the bounding box of the triangle

  return BBox();
}

bool Triangle::intersect(const Ray& r) const {
  return intersect(r, NULL);
}

bool Triangle::intersect(const Ray& r, Intersection* isect) const {
  Vector3D p0 = mesh->positions[v1],
           p1 = mesh->positions[v2],
           p2 = mesh->positions[v3];
  Vector3D n0 = mesh->normals[v1],
           n1 = mesh->normals[v2],
           n2 = mesh->normals[v3];
  Vector3D e1 = p1 - p0,
           e2 = p2 - p0,
           s = r.o - p0;

  Vector3D e1_c_d = cross(e1, r.d);
  Vector3D s_c_e2 = cross(s, e2);
  double denominator = dot(e1_c_d, e2);
  if (abs(denominator) < std::numeric_limits<double>::min())
    return false;

  Vector3D nominator = Vector3D(
          - dot(s_c_e2, r.d),
            dot((e1_c_d), s),
          - dot(s_c_e2, e1)
  );
  Vector3D res = nominator / denominator;

  if (res.x < 0 || res.x > 1.0)
    return false;
  if (res.y < 0 || res.y > 1.0)
    return false;
  if (res.x + res.y > 1)
    return false;
  if (res.z < r.min_t || res.z > r.max_t)
    return false;

  // a hit is found
  r.max_t = res.z;
  if (isect != NULL) {
    isect->t = res.z;
    isect->primitive = this;
    isect->bsdf = mesh->get_bsdf();
    Vector3D n = (1-res.x-res.y)*n0 + res.x*n1 + res.y*n2;
    if (dot(n, r.d) > 0) {
      n = -n;
    }
    isect->n = n;
  }
  return true;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

}  // namespace StaticScene
}  // namespace CMU462
