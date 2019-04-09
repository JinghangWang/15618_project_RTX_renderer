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

  BBox root_bb;
  for (size_t i = 0; i < primitives.size(); ++i) {
    root_bb.expand(primitives[i]->get_bbox());
  }
  root = new BVHNode(root_bb, 0, primitives.size());

  vector<BVHNode*> stack;
  stack.push_back(root);
  while (!stack.empty()) {
    BVHNode* node = stack.back();
    stack.pop_back();
    buildBVHRecursively(node, max_leaf_size);
    if (!node->isLeaf()) {
      stack.push_back(node->l);
      stack.push_back(node->r);
    }
  }
}

void BVHAccel::buildBVHRecursively(BVHNode* root, size_t max_leaf_size) {
  // leaf node -> done
  if (root->range <= max_leaf_size)
    return;
//  cout << "current BVH size " << root->range << endl;

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
  Axis best_dimension = Axis::U;
  size_t best_partition;
  double cost;
  double best_cost = (double) root->range;

  // X direction
  sortPrimitivesInDimension(primitives, start, end, Axis::X);
  // assign primitives to buckets
  double stride = (x_max - x_min)/NUM_BUCKETS;
  for (auto i = start; i < end; ++i) {
    Vector3D centroid = getPrimitiveCentroid(primitives[i]);
    size_t bucket_id = (size_t) ((centroid.x - x_min) / stride);
    buckets[bucket_id].addPrimitive(primitives[i], i);
  }

  // iterate and get best partition
  for (auto i = 0; i < NUM_BUCKETS - 1; ++i) {
    cost = SAH(root_bb, buckets, i);
    if (cost < best_cost) {
      best_cost = cost;
      best_partition = i;
      best_dimension = Axis::X;
    }
  }

  // Y direction
  for (auto& b : buckets) {b.clear();}
  sortPrimitivesInDimension(primitives, start, end, Axis::Y);
  // assign primitives to buckets
  stride = (y_max - y_min)/NUM_BUCKETS;
  for (auto i = start; i < end; ++i) {
    Vector3D centroid = getPrimitiveCentroid(primitives[i]);
    size_t bucket_id = (size_t) ((centroid.y - y_min) / stride);
    buckets[bucket_id].addPrimitive(primitives[i], i);
  }

  // iterate and get best partition
  for (auto i = 0; i < NUM_BUCKETS - 1; ++i) {
    cost = SAH(root_bb, buckets, i);
    if (cost < best_cost) {
      best_cost = cost;
      best_partition = i;
      best_dimension = Axis::Y;
    }
  }

  // Z direction
  for (auto& b : buckets) {b.clear();}
  sortPrimitivesInDimension(primitives, start, end, Axis::Z);
  // assign primitives to buckets
  stride = (z_max - z_min)/NUM_BUCKETS;
  for (auto i = start; i < end; ++i) {
    Vector3D centroid = getPrimitiveCentroid(primitives[i]);
    size_t bucket_id = (size_t) ((centroid.z - z_min) / stride);
    buckets[bucket_id].addPrimitive(primitives[i], i);
  }

  // iterate and get best partition
  for (auto i = 0; i < NUM_BUCKETS - 1; ++i) {
    cost = SAH(root_bb, buckets, i);
    if (cost < best_cost) {
      best_cost = cost;
      best_partition = i;
      best_dimension = Axis::Z;
    }
  }

  // ***** Split  *****
  BVHNode* l = new BVHNode(),
         * r = new BVHNode();
//  cout << "axis " << best_dimension << endl;
  switch (best_dimension){
    case Axis::X: {
//      cout << "X: partition " << best_partition << ", SAH is " << best_cost << endl;
      double partition_line = x_min + (x_max - x_min) / NUM_BUCKETS * (best_partition + 1);

      sortPrimitivesInDimension(primitives, start, end, Axis::X);
      for (auto i = start; i < end; ++i) {
        Vector3D centroid = getPrimitiveCentroid(primitives[i]);
        if (centroid.x < partition_line)
          l->addPrimitive(primitives[i], i);
        else
          r->addPrimitive(primitives[i], i);
      }
      break;
    }
    case Axis::Y: {
//      cout << "Y: partition " << best_partition  << ", SAH is " << best_cost << endl;
      double partition_line = y_min + (y_max - y_min) / NUM_BUCKETS * (best_partition + 1);

      sortPrimitivesInDimension(primitives, start, end, Axis::Y);
      for (auto i = start; i < end; ++i) {
        Vector3D centroid = getPrimitiveCentroid(primitives[i]);
        if (centroid.y < partition_line)
          l->addPrimitive(primitives[i], i);
        else
          r->addPrimitive(primitives[i], i);
      }
      break;
    }
    case Axis::Z: {
//      cout << "Z: partition " << best_partition << ", SAH is " << best_cost << endl;
      double partition_line = z_min + (z_max - z_min) / NUM_BUCKETS * (best_partition + 1);

      sortPrimitivesInDimension(primitives, start, end, Axis::Z);
      for (auto i = start; i < end; ++i) {
        Vector3D centroid = getPrimitiveCentroid(primitives[i]);
        if (centroid.z < partition_line)
          l->addPrimitive(primitives[i], i);
        else
          r->addPrimitive(primitives[i], i);
      }
      break;
    }
    default:
      cout << "No partition is found!" << endl;
      for (auto i = start; i < end; ++i) {
        cout << primitives[i]->get_bbox() << endl;
      }
  }

//  cout << l->range << " " << r->range << endl;
  root->l = l;
  root->r = r;
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

void BVHAccel::sortPrimitivesInDimension(std::vector<Primitive*>& primitives, size_t start, size_t end, Axis dim) {
  if (dim == Axis::X)
    std::sort(
            primitives.begin() + start,
            primitives.begin() + end,
            [] (Primitive* p1, Primitive* p2) -> bool {
                return getPrimitiveCentroid(p1).x < getPrimitiveCentroid(p2).x;
            }
    );
  else if (dim == Axis::Y)
    std::sort(
            primitives.begin() + start,
            primitives.begin() + end,
            [] (Primitive* p1, Primitive* p2) -> bool {
                return getPrimitiveCentroid(p1).y < getPrimitiveCentroid(p2).y;
            }
    );
  else if (dim == Axis::Z)
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
  vector<BVHNode*> stack;
  stack.push_back(root);
  while (!stack.empty()) {
    BVHNode* bvh = stack.back();
    stack.pop_back();
    if (!bvh->isLeaf()) {
      stack.push_back(bvh->l);
      stack.push_back(bvh->r);
    }
    delete bvh;
  }
}

BBox BVHAccel::get_bbox() const { return root->bb; }

bool BVHAccel::intersect(const Ray &ray) const {
  Intersection _isect;
  return intersectHelper(ray, root, &_isect);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *isect) const {
  return intersectHelper(ray, root, isect);
}

bool BVHAccel::intersectHelper(const Ray &ray, const BVHNode* node, Intersection* isect) const {
  if (node->isLeaf()) {
    return intersectLeafNode(ray, node, isect);
  } else {
    double hit1 = node->l->bb.intersect(ray);
    double hit2 = node->r->bb.intersect(ray);
    BVHNode *first = NULL,
            *second = NULL;
    if (hit1 != INF_D && hit2 != INF_D) {
      if (hit1 <= hit2) {
        first = node->l;
        second = node->r;
      } else {
        first = node->r;
        second = node->l;
        std::swap(hit1, hit2);
      }
    } else if (hit1 != INF_D) {
      first = node->l;
    } else if (hit2 != INF_D) {
      first = node->r;
    }

    bool hit = false;
    if (first != NULL)
      hit = intersectHelper(ray, first, isect);
    if (second != NULL && hit2 < isect->t)
      hit |= intersectHelper(ray, second, isect);
    return hit;
  }
}

bool BVHAccel::intersectLeafNode(const Ray &ray, const BVHNode* node, Intersection* isect) const {
  bool hit = false;
  for (auto i = node->start; i < node->start + node->range; ++i) {
    if (primitives[i]->intersect(ray, isect)) hit = true;
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
