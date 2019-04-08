#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"
#include "static_scene/sphere.h"

#include <iostream>
#include <stack>
#include <algorithm>

using namespace std;

namespace CMU462 {
namespace StaticScene {

#define NUM_BUCKETS 4
#define C_SECT 1


BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {
  this->primitives = _primitives;

  // TODO (PathTracer):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox root_bb;
  for (size_t i = 0; i < primitives.size(); ++i) {
    root_bb.expand(primitives[i]->get_bbox());
  }
  root = new BVHNode(root_bb, 0, primitives.size());

  buildBVHRecursively(root, max_leaf_size);
}

void BVHAccel::buildBVHRecursively(BVHNode* root, size_t max_leaf_size) {
  // leaf node -> done
  if (root->range <= max_leaf_size)
    return;

  const BBox& root_bb = root->bb;
  size_t start = root->start,
         end = start + root->range;
  double x_min = root_bb.min.x,
         y_min = root_bb.min.y,
         z_min = root_bb.min.z,
         x_max = root_bb.max.x,
         y_max = root_bb.max.y,
         z_max = root_bb.max.z;
  // init bucket list as BVHNodes
  std::vector<BVHNode> buckets(NUM_BUCKETS);

  // ***** find best partition *****
  char best_dimension;
  size_t best_partition;
  double best_cost = std::numeric_limits<double>::max();

  // X direction
  sortPrimitivesInDimension(primitives, start, end, 'x');
  // assign primitives to buckets
  double stride = (x_max - x_min)/NUM_BUCKETS;
  for (auto i = start; i < end; ++i) {
    Vector3D centroid = getPrimitiveCentroid(primitives[i]);
    size_t bucket_id = (size_t) ((centroid.x - x_min) / stride);
    buckets[bucket_id].addPrimitive(primitives[i], i);
  }

  // iterate and get best partition
  for (auto i = 0; i < NUM_BUCKETS - 1; ++i) {
    double cost = SAH(root_bb, buckets, i);
//    cout << "x: partition " << i << " with SAH " << cost << endl;
    if (cost < best_cost) {
      best_cost = cost;
      best_partition = i;
      best_dimension = 'x';
    }
  }

  // Y direction
  for (auto& b : buckets) {b.clear();}
  sortPrimitivesInDimension(primitives, start, end, 'y');
  // assign primitives to buckets
  stride = (y_max - y_min)/NUM_BUCKETS;
  for (auto i = start; i < end; ++i) {
    Vector3D centroid = getPrimitiveCentroid(primitives[i]);
    size_t bucket_id = (size_t) ((centroid.y - y_min) / stride);
    buckets[bucket_id].addPrimitive(primitives[i], i);
  }

  // iterate and get best partition
  for (auto i = 0; i < NUM_BUCKETS - 1; ++i) {
    double cost = SAH(root_bb, buckets, i);
//    cout << "y: partition " << i << " with SAH " << cost << endl;
    if (cost < best_cost) {
      best_cost = cost;
      best_partition = i;
      best_dimension = 'y';
    }
  }

  // ***** Split  *****
  BVHNode* l = new BVHNode(),
         * r = new BVHNode();

  if (best_dimension == 'x') {
    cout << "X: partition plane is " << best_partition  << ", SAH is " << best_cost << endl;
    double partition_line = x_min + (x_max - x_min) / NUM_BUCKETS * (best_partition + 1);

    sortPrimitivesInDimension(primitives, start, end, 'x');
    for (auto i = start; i < end; ++i) {
      Vector3D centroid = getPrimitiveCentroid(primitives[i]);
      if (centroid.x < partition_line)
        l->addPrimitive(primitives[i], i);
      else
        r->addPrimitive(primitives[i], i);
    }
  } else if (best_dimension == 'y') {
    cout << "Y: partition plane is " << best_partition  << ", SAH is " << best_cost << endl;
    double partition_line = y_min + (y_max - y_min) / NUM_BUCKETS * (best_partition + 1);

    sortPrimitivesInDimension(primitives, start, end, 'y');
    for (auto i = start; i < end; ++i) {
      Vector3D centroid = getPrimitiveCentroid(primitives[i]);
      if (centroid.y < partition_line)
        l->addPrimitive(primitives[i], i);
      else
        r->addPrimitive(primitives[i], i);
    }
  }
  root->l = l;
  root->r = r;
  buildBVHRecursively(l, max_leaf_size);
  buildBVHRecursively(r, max_leaf_size);
}

// retrieves primitive centroid if it is either
// a triangle or a sphere
Vector3D BVHAccel::getPrimitiveCentroid(Primitive* t) {
  if (dynamic_cast<Triangle*>(t)) {
    Triangle* tri = static_cast<Triangle*>(t);
    return tri->getCentroid();
  } else if (dynamic_cast<Sphere*>(t)) {
    Sphere* sph = static_cast<Sphere*>(t);
    return sph->getCentroid();
  }
  return Vector3D();
}

void BVHAccel::sortPrimitivesInDimension(std::vector<Primitive*>& primitives, size_t start, size_t end, char dim) {
  if (dim == 'x')
    std::sort(
            primitives.begin() + start,
            primitives.begin() + end,
            [] (Primitive* p1, Primitive* p2) -> bool {
                return getPrimitiveCentroid(p1).y < getPrimitiveCentroid(p2).y;
            }
    );
  else if (dim == 'y')
    std::sort(
            primitives.begin() + start,
            primitives.begin() + end,
            [] (Primitive* p1, Primitive* p2) -> bool {
                return getPrimitiveCentroid(p1).y < getPrimitiveCentroid(p2).y;
            }
    );
  else if (dim == 'z')
    std::sort(
            primitives.begin() + start,
            primitives.begin() + end,
            [] (Primitive* p1, Primitive* p2) -> bool {
                return getPrimitiveCentroid(p1).z < getPrimitiveCentroid(p2).z;
            }
    );
}

double BVHAccel::SAH(const BBox& parent_bb,const std::vector<BVHNode>& buckets, size_t partition_plane) {
  BBox bb_l, bb_r;
  size_t num_l = 0, num_r = 0;
  for (auto i = 0; i < partition_plane + 1; ++i) {
    bb_l.expand(buckets[i].bb);
    num_l += buckets[i].range;
  }

  for (auto i = partition_plane + 1; i < buckets.size(); ++i) {
    bb_r.expand(buckets[i].bb);
    num_r += buckets[i].range;
  }
  double cost = 0;
  cost += bb_l.surface_area()/parent_bb.surface_area() * num_l * C_SECT;
  cost += bb_r.surface_area()/parent_bb.surface_area() * num_r * C_SECT;
  return cost;
}

BVHAccel::~BVHAccel() {
  // TODO (PathTracer):
  // Implement a proper destructor for your BVH accelerator aggregate

}

BBox BVHAccel::get_bbox() const { return root->bb; }

bool BVHAccel::intersect(const Ray &ray) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate.

  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if (primitives[p]->intersect(ray)) hit = true;
  }

  return hit;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *isect) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.

  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if (primitives[p]->intersect(ray, isect)) hit = true;
  }

  return hit;
}

void BVHNode::clear() {
  bb = BBox();
  start = std::numeric_limits<size_t>::max();
  range = 0;
  l = NULL;
  r = NULL;
}

void BVHNode::addPrimitive(const Primitive* t, size_t i) {
  if (i < start)
    start = i;
  range += 1;
  bb.expand(t->get_bbox());
}

}  // namespace StaticScene
}  // namespace CMU462
