#include <float.h>
#include <assert.h>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"
#include "error_dialog.h"
#include <assert.h>

#define CLOSE_TO_ZERO 0.0000001L // TODO

namespace CMU462 {

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should split the given edge and return an iterator to the
  // newly inserted vertex. The halfedge of this vertex should point along
  // the edge that was split, rather than the new edges.

  showError("splitEdge() not implemented.");
  return VertexIter();
}

VertexIter HalfedgeMesh::collapseEdge(EdgeIter e) {
  // not collapse edge at boundary
  if (e->isBoundary()) {
    return e->halfedge()->vertex();
  }
  HalfedgeIter h0 = e->halfedge(),
               h1 = h0->twin();
  VertexIter v0 = h0->vertex(),
             v1 = h1->vertex();
  FaceIter f0 = h0->face(),
           f1 = h1->face();

  vector<HalfedgeIter> halfedges_from_v0,
                       halfedges_from_v1;

  // collect edges "from" both endpoints
  // from edges from v0 starts h1's next
  HalfedgeIter cur_halfedge = h1->next();
  do {
    halfedges_from_v0.push_back(cur_halfedge);
    cur_halfedge = cur_halfedge->twin()->next();
  } while (cur_halfedge != h0);

  // from edges from v1 starts h0's next
  cur_halfedge = h0->next();
  do {
    halfedges_from_v1.push_back(cur_halfedge);
    cur_halfedge = cur_halfedge->twin()->next();
  } while (cur_halfedge != h1);

  // check whether neighboring faces are triangles
  bool f0_was_triangle = !f0->isPolygon(),
       f1_was_triangle = !f1->isPolygon();

   // allocate new vertex
  VertexIter v = newVertex();
  v->position = e->centroid();

  // assign
  // Halfedges
  for (auto h : halfedges_from_v0) { h->vertex() = v;}
  for (auto h : halfedges_from_v1) { h->vertex() = v;}
  // next edges of prev halfedges of h0/h1 should be the first in the halfedges_from
  (*prev(halfedges_from_v0.end()))->twin()->next() = halfedges_from_v1.front();
  (*prev(halfedges_from_v1.end()))->twin()->next() = halfedges_from_v0.front();

  // faces
  f0->halfedge() = *halfedges_from_v1.begin();
  f1->halfedge() = *halfedges_from_v0.begin();

  // vertex
  v->halfedge() = *halfedges_from_v0.begin();

  // remove elements in neighboring faces that were triangles
  if (f0_was_triangle) {
    HalfedgeIter toremove_h0 = h0->next(),
                 toremove_h1 = toremove_h0->next();
    removeFaceWithTwoEdges(toremove_h0, toremove_h1);
  }
  if (f1_was_triangle) {
    HalfedgeIter toremove_h0 = h1->next(),
                 toremove_h1 = toremove_h0->next();
    removeFaceWithTwoEdges(toremove_h0, toremove_h1);
  }

  // remove elements
  deleteVertex(v0);
  deleteVertex(v1);
  deleteEdge(e);
  deleteHalfedge(h0);
  deleteHalfedge(h1);
  return v;
}

VertexIter HalfedgeMesh::collapseFace(FaceIter f) {
  // TODO: (meshEdit)
  // This method should collapse the given face and return an iterator to
  // the new vertex created by the collapse.
  showError("collapseFace() not implemented.");
  return VertexIter();
}

FaceIter HalfedgeMesh::eraseVertex(VertexIter v) {
  vector<HalfedgeIter> halfedges_from_v,
                       halfedges_to_v;
  vector<FaceIter> neighboring_faces;
  vector<EdgeIter> edges_to_v;

  // collect edges, halfedges from and to the vertex v
  HalfedgeIter h0 = v->halfedge();
  HalfedgeIter cur_h = h0;
  do{
    halfedges_from_v.push_back(cur_h);
    halfedges_to_v.push_back(cur_h->twin());
    neighboring_faces.push_back(cur_h->face());
    cur_h = cur_h->twin()->next();
  } while (cur_h != h0);

  // quit if any of the neighboring face is at boundary
  for (auto f : neighboring_faces) {
    if (f->isBoundary()) {
      return FaceIter();
    }
  }

  // create a new face
  FaceIter new_f = newFace();
  new_f->halfedge() = halfedges_from_v.front()->next();

  // iterate surrounding halfedges
  // assign their face to the newly created face
  for (auto h : halfedges_from_v) {
    cur_h = h;
    do {
      cur_h->face() = new_f;
      cur_h = cur_h->next();
    } while (cur_h != h);
  }

  // connect outter halfedges
  // assign vertices' halfedges correctly
  for (auto h : halfedges_to_v) {
    VertexIter vertex = h->vertex();
    HalfedgeIter outter_h = h->prev();
    outter_h->next() = h->twin()->next();
    vertex->halfedge() = h->twin()->next();
  }

  // remove all edges/halfedges/faces collected, and the vertex
  for (auto h : halfedges_from_v) {
    deleteEdge(h->edge());
    deleteHalfedge(h);
  }
  for (auto h : halfedges_to_v) {
    deleteHalfedge(h);
  }
  for (auto f : neighboring_faces) {
    deleteFace(f);
  }
  deleteVertex(v);

  return new_f;
}

FaceIter HalfedgeMesh::eraseEdge(EdgeIter e) {
  HalfedgeIter& h0 = e->halfedge(),
              & h1 = h0->twin();
  FaceIter f0 = h0->face(),
           f1 = h1->face();

  // return original non-boundary face if edge is at boundary
  if (e->isBoundary()) {
    if (h0->isBoundary())
      return f1;
    return f0;
  }

  // return face if both faces are the same one
  if (f0 == f1)
    return f0;

  // erase regular edge
  VertexIter v0 = h0->vertex(),
             v1 = h1->vertex();

  HalfedgeIter h3 = h1->next(),
               h5 = h1->prev(),
               h2 = h0->next(),
               h4 = h0->prev();

  // Halfedges
  // 1. reassign face from f1 to f0
  HalfedgeIter cur_h = h1;
  do {
    cur_h->face() = f0;
    cur_h = cur_h->next();
  } while (cur_h != h1);

  // 2. fix nexts
  h4->next() = h3;
  h5->next() = h2;


  // Vertices
  v0->halfedge() = h3;
  v1->halfedge() = h2;

  // Face
  f0->halfedge() = h2;

  // delete f1 and keep f0
  deleteFace(f1);
  deleteHalfedge(h0);
  deleteHalfedge(h1);
  deleteEdge(e);

  return f0;
}

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // return if edge is a boundary edge
  if (e0->isBoundary()) return e0;

  // 1. Collect elements
  // HALFEDGES
  HalfedgeIter h0 = e0->halfedge(),
               h1 = h0->next(),
               h2 = h0->prev(),
               h3 = h0->twin(),
               h4 = h3->next(),
               h5 = h3->prev(),
               h6 = h1->twin(),
               h7 = h2->twin(),
               h8 = h4->twin(),
               h9 = h5->twin(),
               h10 = h1->next(),
               h11 = h4->next();

  // VERTICES
  VertexIter v0 = h0->vertex(),
             v1 = h3->vertex(),
             v2 = h6->vertex(),
             v3 = h2->vertex(),
             v4 = h8->vertex(),
             v5 = h5->vertex();

  // EDGES
  EdgeIter e1 = h5->edge(),
           e2 = h4->edge(),
           e3 = h2->edge(),
           e4 = h1->edge();

  // FACES
  FaceIter f0 = h0->face(),
           f1 = h3->face();

  // 2. Reassign elements
  // HALFEDGES
  h0->next() = h10;
  h0->twin() = h3;
  h0->vertex() = v4;
  h0->edge() = e0;
  h0->face() = f0;
  h1->next() = h3;
  h1->twin() = h6;
  h1->vertex() = v1;
  h1->edge() = e4;
  h1->face() = f1;
  h2->next() = h4;
  h2->twin() = h7;
  h2->vertex() = v3;
  h2->edge() = e3;
  h2->face() = f0;
  h3->next() = h11;
  h3->twin() = h0;
  h3->vertex() = v2;
  h3->edge() = e0;
  h3->face() = f1;
  h4->next() = h0;
  h4->twin() = h8;
  h4->vertex() = v0;
  h4->edge() = e2;
  h4->face() = f0;
  h5->next() = h1;
  h5->twin() = h9;
  h5->vertex() = v5;
  h5->edge() = e1;
  h5->face() = f1;
  h6->next() = h6->next();
  h6->twin() = h1;
  h6->vertex() = v2;
  h6->edge() = e4;
  h6->face() = h6->face();
  h7->next() = h7->next();
  h7->twin() = h2;
  h7->vertex() = v0;
  h7->edge() = e3;
  h7->face() = h7->face();
  h8->next() = h8->next();
  h8->twin() = h4;
  h8->vertex() = v4;
  h8->edge() = e2;
  h8->face() = h8->face();
  h9->next() = h9->next();
  h9->twin() = h5;
  h9->vertex() = v1;
  h9->edge() = e1;
  h9->face() = h9->face();

  // VERTICES
  v0->halfedge() = h4;
  v1->halfedge() = h1;
  v2->halfedge() = h6;
  v3->halfedge() = h2;
  v4->halfedge() = h0;
  v5->halfedge() = h5;

  // EDGES
  e0->halfedge() = h0;
  e1->halfedge() = h5;
  e2->halfedge() = h4;
  e3->halfedge() = h2;
  e4->halfedge() = h1;

  // FACES
  f0->halfedge() = h0;
  f1->halfedge() = h3;

  return e0;
}

void HalfedgeMesh::subdivideQuad(bool useCatmullClark) {
  // Unlike the local mesh operations (like bevel or edge flip), we will perform
  // subdivision by splitting *all* faces into quads "simultaneously."  Rather
  // than operating directly on the halfedge data structure (which as you've
  // seen
  // is quite difficult to maintain!) we are going to do something a bit nicer:
  //
  //    1. Create a raw list of vertex positions and faces (rather than a full-
  //       blown halfedge mesh).
  //
  //    2. Build a new halfedge mesh from these lists, replacing the old one.
  //
  // Sometimes rebuilding a data structure from scratch is simpler (and even
  // more
  // efficient) than incrementally modifying the existing one.  These steps are
  // detailed below.

  // Step I: Compute the vertex positions for the subdivided mesh.  Here
  // we're
  // going to do something a little bit strange: since we will have one vertex
  // in
  // the subdivided mesh for each vertex, edge, and face in the original mesh,
  // we
  // can nicely store the new vertex *positions* as attributes on vertices,
  // edges,
  // and faces of the original mesh.  These positions can then be conveniently
  // copied into the new, subdivided mesh.
  if (useCatmullClark) {
    computeCatmullClarkPositions();
  } else {
    computeLinearSubdivisionPositions();
  }

  // Step II: Assign a unique index (starting at 0) to each vertex, edge,
  // and
  // face in the original mesh.  These indices will be the indices of the
  // vertices
  // in the new (subdivided mesh).  They do not have to be assigned in any
  // particular
  // order, so long as no index is shared by more than one mesh element, and the
  // total number of indices is equal to V+E+F, i.e., the total number of
  // vertices
  // plus edges plus faces in the original mesh.  Basically we just need a
  // one-to-one
  // mapping between original mesh elements and subdivided mesh vertices.
  // [See subroutine for actual
  assignSubdivisionIndices();

  // Step III: Build a list of quads in the new (subdivided) mesh, as
  // tuples of
  // the element indices defined above.  In other words, each new quad should be
  // of
  // the form (i,j,k,l), where i,j,k and l are four of the indices stored on our
  // original mesh elements.  Note that it is essential to get the orientation
  // right
  // here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces should
  // circulate in the same direction as old faces (think about the right-hand
  // rule).
  // [See subroutines for actual
  vector<vector<Index> > subDFaces;
  vector<Vector3D> subDVertices;
  buildSubdivisionFaceList(subDFaces);
  buildSubdivisionVertexList(subDVertices);

  // Step IV: Pass the list of vertices and quads to a routine that clears
  // the
  // internal data for this halfedge mesh, and builds new halfedge data from
  // scratch,
  // using the two lists.
  rebuild(subDFaces, subDVertices);
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * simple linear interpolation, e.g., the edge midpoints and face
 * centroids.
 */
void HalfedgeMesh::computeLinearSubdivisionPositions() {
  // For each vertex, assign Vertex::newPosition to
  // its original position, Vertex::position.
  for (auto& v : vertices) {
    v.newPosition = v.position;
  }

  // For each edge, assign the midpoint of the two original
  // positions to Edge::newPosition.
  for (auto& e : edges) {
    e.newPosition = e.centroid();
  }

  // For each face, assign the centroid (i.e., arithmetic mean)
  // of the original vertex positions to Face::newPosition.  Note
  // that in general, NOT all faces will be triangles!
  for (auto& f : faces) {
    f.newPosition = f.centroid();
  }
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * the Catmull-Clark rules for subdivision.
 */
void HalfedgeMesh::computeCatmullClarkPositions() {
  // The implementation for this routine should be
  // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
  // except that the calculation of the positions themsevles is
  // slightly more involved, using the Catmull-Clark subdivision
  // rules. (These rules are outlined in the Developer Manual.)

  // face
  for (auto& f : faces) {
    f.newPosition = f.centroid();
  }

  // edges
  for (auto& e : edges) {
    if (e.isBoundary()) {
      e.newPosition = e.centroid();
    } else {
      HalfedgeIter h0 = e.halfedge(),
              h1 = h0->twin();
      VertexIter v0 = h0->vertex(),
              v1 = h1->vertex();
      FaceIter f0 = h0->face(),
              f1 = h1->face();
      e.newPosition = (v0->position + v1->position + f0->newPosition + f1->newPosition) / 4;
    }
  }

  // vertices
  for (auto& v : vertices) {
    if (v.isBoundary()) { // handle boundary vertices
       v.newPosition = v.position;
      continue;
    }

    Vector3D Q, R;
    Size degree = 0;
    HalfedgeIter h = v.halfedge();
    do {
      degree++;
      Q += h->face()->newPosition; // newPosition of face == centroid
      R += h->edge()->centroid();
      h = h->twin()->next();
    } while (h != v.halfedge());

    Q /= degree;
    R /= degree;
    v.newPosition = (Q + 2*R + (degree - 3) * v.position)/degree;
  }
}

/**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
void HalfedgeMesh::assignSubdivisionIndices() {
  // Start a counter at zero; if you like, you can use the
  // "Index" type (defined in halfedgeMesh.h)
  Index i = 0;

  // Iterate over vertices, assigning values to Vertex::index
  for (auto& v : vertices) {
    v.index = i++;
  }

  // Iterate over edges, assigning values to Edge::index
  for (auto& e : edges) {
    e.index = i++;
  }

  // Iterate over faces, assigning values to Face::index
  for (auto& f : faces) {
    f.index = i++;
  }
}

/**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D>& subDVertices) {
  // Resize the vertex list so that it can hold all the vertices.
  subDVertices.resize(vertices.size() + edges.size() + faces.size());

  // Iterate over vertices, assigning Vertex::newPosition to the
  // appropriate location in the new vertex list.
  for (auto& v : vertices) {
    subDVertices[v.index] = v.newPosition;
  }

  // Iterate over edges, assigning Edge::newPosition to the appropriate
  // location in the new vertex list.
  for (auto& e : edges) {
    subDVertices[e.index] = e.newPosition;
  }

  // Iterate over faces, assigning Face::newPosition to the appropriate
  // location in the new vertex list.
  for (auto& f : faces) {
    subDVertices[f.index] = f.newPosition;
  }
}

/**
 * Build a flat list containing all the quads in a Catmull-Clark
 * (or linear) subdivision of this mesh.  Each quad is specified
 * by a vector of four indices (i,j,k,l), which come from the
 * members Vertex::index, Edge::index, and Face::index.  Note that
 * the ordering of these indices is important because it determines
 * the orientation of the new quads; it is also important to avoid
 * "bowties."  For instance, (l,k,j,i) has the opposite orientation
 * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
 * will look like a bowtie.
 */
void HalfedgeMesh::buildSubdivisionFaceList(vector<vector<Index> >& subDFaces) {
  // This routine is perhaps the most tricky step in the construction of
  // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
  // vertex positions).  Basically what you want to do is iterate over faces,
  // then for each face, append N quads to the list (where N is the
  // degree of the face).  For this routine, it may be more convenient to simply
  // append quads to the end of the list (rather than allocating it ahead of
  // time), though YMMV.  You can of course iterate around a face by starting
  // with its first halfedge and following the "next" pointer until you get
  // back to the beginning.  The tricky part is making sure you grab the right
  // indices in the right order---remember that there are indices on vertices,
  // edges, AND faces of the original mesh.  All of these should get used.  Also
  // remember that you must have FOUR indices per face, since you are making a
  // QUAD mesh!

  //  iterate over faces
  for (auto f : faces) {
    // loop around face
    HalfedgeIter h = f.halfedge();
    do {
      // build lists of four indices for each sub-quad
      vector<Index> quad(4);
      quad[0] = f.index;
      quad[1] = h->edge()->index;
      h = h->next();
      quad[2] = h->vertex()->index;
      quad[3] = h->edge()->index;
      // append each list of four indices to face list
      subDFaces.push_back(quad);
    } while (h != f.halfedge());
  }
}

FaceIter HalfedgeMesh::bevelVertex(VertexIter v) {
  // TODO This method should replace the vertex v with a face, corresponding to
  // a bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelVertexComputeNewPositions (which you also have to
  // implement!)

  showError("bevelVertex() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelEdge(EdgeIter e) {
  // TODO This method should replace the edge e with a face, corresponding to a
  // bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelEdgeComputeNewPositions (which you also have to
  // implement!)

  showError("bevelEdge() not implemented.");
  return facesBegin();
}

// This method should replace the face f with an additional, inset face
// (and ring of faces around it), corresponding to a bevel operation.
FaceIter HalfedgeMesh::bevelFace(FaceIter f) {
  if (f->isBoundary()) return f; // if on boundary, return
  vector<HalfedgeIter> orig_halfedges, inner_halfedges, outer_halfedges;
  vector<HalfedgeIter> diag_halfedges_from, diag_halfedges_to; // diagonal halfedges from or to new vertices
  vector<VertexIter> inner_vertices, outer_vertices;
  vector<FaceIter> faces;
  vector<EdgeIter> ring_edges, diag_edges;

  HalfedgeIter cur_halfedge = f->halfedge();
  do {
    orig_halfedges.push_back(cur_halfedge);
    outer_vertices.push_back(cur_halfedge->vertex());
    cur_halfedge = cur_halfedge->next();
  } while (cur_halfedge != f->halfedge());
  int num_edges = orig_halfedges.size();

  // allocate new vertices, ring of faces, ring of edges, diagonal edges and halfedges
  for (int i = 0; i < num_edges; ++i) {
    faces.push_back(newFace());
    inner_vertices.push_back(newVertex());
    ring_edges.push_back(newEdge());
    diag_edges.push_back(newEdge());
    inner_halfedges.push_back(newHalfedge());
    outer_halfedges.push_back(newHalfedge());
    diag_halfedges_from.push_back(newHalfedge());
    diag_halfedges_to.push_back(newHalfedge());
  }

  // construct connectivity
  for (int i = 0; i < num_edges; ++i) {
    int idx_next = (i + 1) % num_edges;
    int idx_prev = (i + num_edges - 1) % num_edges;
    // collect
    FaceIter& ring_face = faces[i];
    FaceIter& prev_ring_face = faces[idx_prev];
    VertexIter& inner_vertex = inner_vertices[i];
    VertexIter& outer_vertex = outer_vertices[i];
    VertexIter& next_inner_vertex = inner_vertices[idx_next];
    HalfedgeIter& outer_halfedge = outer_halfedges[i];
    HalfedgeIter& inner_halfedge = inner_halfedges[i];
    HalfedgeIter& prev_outer_halfedge = outer_halfedges[idx_prev];
    HalfedgeIter& next_inner_halfedge = inner_halfedges[idx_next];
    HalfedgeIter& orig_halfedge = orig_halfedges[i];
    HalfedgeIter& diag_halfedge_from = diag_halfedges_from[i];
    HalfedgeIter& diag_halfedge_to = diag_halfedges_to[i];
    HalfedgeIter& next_diag_halfedge_to = diag_halfedges_to[idx_next];
    EdgeIter& diag_edge = diag_edges[i];
    EdgeIter& ring_edge = ring_edges[i];
    EdgeIter& orig_edge = orig_halfedge->edge();

    // assign
    // HALFEDGE
    outer_halfedge->next() = diag_halfedge_from;
    outer_halfedge->twin() = inner_halfedge;
    outer_halfedge->vertex() = next_inner_vertex;
    outer_halfedge->edge() = ring_edge;
    outer_halfedge->face() = ring_face;
    inner_halfedge->next() = next_inner_halfedge;
    inner_halfedge->twin() = outer_halfedge;
    inner_halfedge->vertex() = inner_vertex;
    inner_halfedge->edge() = ring_edge;
    inner_halfedge->face() = f;
    diag_halfedge_from->next() = orig_halfedge;
    diag_halfedge_from->twin() = diag_halfedge_to;
    diag_halfedge_from->vertex() = inner_vertex;
    diag_halfedge_from->edge() = diag_edge;
    diag_halfedge_from->face() = ring_face;
    diag_halfedge_to->next() = prev_outer_halfedge;
    diag_halfedge_to->twin() = diag_halfedge_from;
    diag_halfedge_to->vertex() = outer_vertex;
    diag_halfedge_to->edge() = diag_edge;
    diag_halfedge_to->face() = prev_ring_face;
    orig_halfedge->next() = next_diag_halfedge_to;
    orig_halfedge->twin() = orig_halfedge->twin(); // same
    orig_halfedge->vertex() = outer_vertex; // should be same
    orig_halfedge->edge() = orig_halfedge->edge(); // same
    orig_halfedge->face() = ring_face;

    // VERTEX
    inner_vertex->halfedge() = inner_halfedge;
    outer_vertex->halfedge() = orig_halfedge;

    // EDGE
    ring_edge->halfedge() = inner_halfedge;
    diag_edge->halfedge() = diag_halfedge_from;

    // FACE
    ring_face->halfedge() = outer_halfedge;
  }

  // CENTER FACE
  f->halfedge() = inner_halfedges.front();

  return f;
}


void HalfedgeMesh::bevelFaceComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double normalShift,
    double tangentialInset) {
  Vector3D center;
  auto num_edges = newHalfedges.size();
  for (auto i = 0; i < num_edges; ++i) {
    center += originalVertexPositions[i];
  }
  center /= num_edges;

  // tangential inset
  for(auto i = 0; i < num_edges; ++i) {
    Vector3D& pi = originalVertexPositions[i]; // get the original vertex
    Vector3D& new_pos = newHalfedges[i]->vertex()->position;
    Vector3D offset = tangentialInset * (center - pi).unit();
    if (new_pos.x == 0.0 && new_pos.y == 0.0 && new_pos.z == 0.0)
      new_pos += pi;
    new_pos += offset;
  }

  // normal shift
  // traverse around to get the center face
  FaceIter center_face = (*newHalfedges.begin())->next()->next()->next()->twin()->face();
  Vector3D normal = center_face->normal();
  for(auto i = 0; i < num_edges; ++i) {
    newHalfedges[i]->vertex()->position += normal * normalShift;
  }

}

void HalfedgeMesh::bevelVertexComputeNewPositions(
    Vector3D originalVertexPosition, vector<HalfedgeIter>& newHalfedges,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled vertex.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., hs.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.

}

void HalfedgeMesh::bevelEdgeComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled edge.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); i++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //

}

void HalfedgeMesh::splitPolygons(vector<FaceIter>& fcs) {
  for (auto f : fcs) splitPolygon(f);
}

void HalfedgeMesh::splitPolygon(FaceIter f) {
  if (f->isBoundary()) return;
  if (!f->isPolygon()) return;

  vector<VertexIter> vertices;
  vector<HalfedgeIter> halfedges;
  HalfedgeIter h0 = f->halfedge();
  HalfedgeIter cur_h = h0->next(); // start from the next halfedge
  VertexIter v0 = h0->vertex();

  // collect all vertices except the first one
  while (cur_h != h0) {
    halfedges.push_back(cur_h);
    vertices.push_back(cur_h->vertex());
    cur_h = cur_h->next();
  }

  HalfedgeIter h1, h2, h3, h4;
  FaceIter new_f;
  // loop to triangulate
  for (auto i = 1; i < vertices.size(); ++i) {
    h1 = halfedges[i - 1];
    h3 = halfedges[i];
    if (i == vertices.size() - 1) { // handle last triangle specially
      h2 = halfedges.back();
      new_f = f;
    } else {
      new_f = newFace();
      h2 = newHalfedge();
      h4 = newHalfedge();
    }
    VertexIter v1 = vertices[i - 1],
               v2 = vertices[i];

    // assign and connect
    // HALFEDGES
    h0->next() = h1;
    h0->face() = new_f;
    h1->next() = h2;
    h1->face() = new_f;
    h2->next() = h0;
    h2->face() = new_f;

    if (i != vertices.size() -1 ) {
      EdgeIter e = newEdge();
      e->halfedge() = h2;

      h2->twin() = h4;
      h2->edge() = e;
      h2->vertex() = v2;

      h4->next() = h3;
      h4->twin() = h2;
      h4->edge() = e;
      h4->vertex() = v0;
    }
    // h4 will be next h0

    // FACE
    new_f->halfedge() = h0;

    // update h0
    h0 = h4;
  }
}

void HalfedgeMesh::removeFaceWithTwoEdges(HalfedgeIter h0,
                                          HalfedgeIter h1) {
  HalfedgeIter remaining_halfedge_0 = h0->twin(),
               remaining_halfedge_1 = h1->twin();
  // allocate new edge and assign
  EdgeIter new_e = newEdge();
  remaining_halfedge_0->twin() = remaining_halfedge_1;
  remaining_halfedge_0->edge() = new_e;
  remaining_halfedge_1->twin() = remaining_halfedge_0;
  remaining_halfedge_1->edge() = new_e;
  new_e->halfedge() = remaining_halfedge_0;

  remaining_halfedge_0->vertex()->halfedge() = remaining_halfedge_0;
  remaining_halfedge_1->vertex()->halfedge() = remaining_halfedge_1;

  // remove face, two halfedges and edge
  FaceIter &toremove_f = h0->face();
  if (toremove_f->isBoundary())
    deleteBoundary(toremove_f);
  deleteFace(toremove_f);
  deleteEdge(h0->edge());
  deleteEdge(h1->edge());
  deleteHalfedge(h0);
  deleteHalfedge(h1);
}

EdgeRecord::EdgeRecord(EdgeIter& _edge) : edge(_edge) {
  // Compute the combined quadric from the edge endpoints.
  HalfedgeIter h0 = _edge->halfedge(),
               h1 = h0->twin();
  VertexIter v0 = h0->vertex(),
             v1 = h1->vertex();
  Matrix4x4 quadric = v0->quadric + v1->quadric;

  // -> Build the 3x3 linear system whose solution minimizes the quadric error
  //    associated with these two endpoints.
  // computed by accumulating quadrics and then extacting the upper-left 3x3 block
  Matrix3x3 A;
  A[0] = quadric[0].to3D();
  A[1] = quadric[1].to3D();
  A[2] = quadric[2].to3D();

  // computed by extracting minus the upper-right 3x1 column from the same matrix
  Vector3D b = - quadric[3].to3D();

  // -> Use this system to solve for the optimal position, and store it in
  //    EdgeRecord::optimalPoint.
  Vector3D x;
  if (abs(A.det()) < CLOSE_TO_ZERO) {
    // A is singular, need to find optimal x on the edge
    unsigned int samples = 100;
    Vector3D e01 = (v1->position - v0->position) / samples;
    Vector3D optimalX;
    double cost = numeric_limits<double>::infinity();
    for (auto i = 0; i < samples+1; ++i) {
      Vector3D current_x = v0->position + i * e01;
      Vector4D xx = Vector4D(current_x, 1.0);
      double new_cost = dot(xx, quadric * xx);
      if (new_cost < cost) {
        cost = new_cost;
        optimalX = current_x;
      }
    }
    x = optimalX;
  } else {
    x = A.inv() * b; // solve Ax = b for x, by hitting both sides with the inverse of A
  }
  optimalPoint = x;

  // -> Also store the cost associated with collapsing this edg in
  //    EdgeRecord::Cost.
  Vector4D xx = Vector4D(x, 1.0);
  score = dot(xx, quadric * xx);
}

void MeshResampler::upsample(HalfedgeMesh& mesh)
// This routine should increase the number of triangles in the mesh using Loop
// subdivision.
{
  // TODO: (meshEdit)
  // Compute new positions for all the vertices in the input mesh, using
  // the Loop subdivision rule, and store them in Vertex::newPosition.
  // -> At this point, we also want to mark each vertex as being a vertex of the
  //    original mesh.
  // -> Next, compute the updated vertex positions associated with edges, and
  //    store it in Edge::newPosition.
  // -> Next, we're going to split every edge in the mesh, in any order.  For
  //    future reference, we're also going to store some information about which
  //    subdivided edges come from splitting an edge in the original mesh, and
  //    which edges are new, by setting the flat Edge::isNew. Note that in this
  //    loop, we only want to iterate over edges of the original mesh.
  //    Otherwise, we'll end up splitting edges that we just split (and the
  //    loop will never end!)
  // -> Now flip any new edge that connects an old and new vertex.
  // -> Finally, copy the new vertex positions into final Vertex::position.

  // Each vertex and edge of the original surface can be associated with a
  // vertex in the new (subdivided) surface.
  // Therefore, our strategy for computing the subdivided vertex locations is to
  // *first* compute the new positions
  // using the connectity of the original (coarse) mesh; navigating this mesh
  // will be much easier than navigating
  // the new subdivided (fine) mesh, which has more elements to traverse.  We
  // will then assign vertex positions in
  // the new mesh based on the values we computed for the original mesh.

  // Compute updated positions for all the vertices in the original mesh, using
  // the Loop subdivision rule.

  // Next, compute the updated vertex positions associated with edges.

  // Next, we're going to split every edge in the mesh, in any order.  For
  // future
  // reference, we're also going to store some information about which
  // subdivided
  // edges come from splitting an edge in the original mesh, and which edges are
  // new.
  // In this loop, we only want to iterate over edges of the original
  // mesh---otherwise,
  // we'll end up splitting edges that we just split (and the loop will never
  // end!)

  // Finally, flip any new edge that connects an old and new vertex.

  // Copy the updated vertex positions to the subdivided mesh.
  showError("upsample() not implemented.");
}

void MeshResampler::downsample(HalfedgeMesh& mesh) {
  // Compute initial quadrics for each face by simply writing the plane equation
  // for the face in homogeneous coordinates. These quadrics should be stored
  // in Face::quadric
  for (auto f = mesh.facesBegin(); f != mesh.facesEnd(); ++f) {
    VertexIter vertex = f->halfedge()->vertex();
    Vector3D normal = f->normal();
    Vector4D v = Vector4D(normal, - dot(normal, vertex->position));
    f->quadric = outer(v, v);
  }

  // -> Compute an initial quadric for each vertex as the sum of the quadrics
  //    associated with the incident faces, storing it in Vertex::quadric
  for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
    HalfedgeIter h = v->halfedge();
    do {
      FaceIter f = h->face();
      if (!f->isBoundary())
        v->quadric += f->quadric;
        h = h->twin()->next();
    } while (h != v->halfedge());
  }

  // -> Build a priority queue of edges according to their quadric error cost,
  //    i.e., by building an EdgeRecord for each edge and sticking it in the
  //    queue.
  MutablePriorityQueue<EdgeRecord> queue;
  for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
    HalfedgeIter h = e->halfedge();
    if (!h->isBoundary() || !h->twin()->isBoundary()) {
      e->record = EdgeRecord(e);
      queue.insert(e->record);
    }
  }

  // -> Until we reach the target edge budget, collapse the best edge. Remember
  //    to remove from the queue any edge that touches the collapsing edge
  //    BEFORE it gets collapsed, and add back into the queue any edge touching
  //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
  //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
  //    top of the queue.
  auto target = mesh.nFaces() / 4;
  while (mesh.nFaces() > target)
  {
    EdgeRecord best_record = queue.top();
    EdgeIter best_edge = best_record.edge;
    queue.pop();

    VertexIter v0 = best_edge->halfedge()->vertex(),
               v1 = best_edge->halfedge()->twin()->vertex();
    Matrix4x4 new_quadric = v0->quadric + v1->quadric;

    // remove edges touching either endpoints
    HalfedgeIter h = v0->halfedge();
    do {
      if (!h->isBoundary() || !h->twin()->isBoundary()) {
        queue.remove(h->edge()->record);
      }
      h = h->twin()->next();
    } while (h != v0->halfedge());
    h = v1->halfedge();
    do {
      if (!h->isBoundary() || !h->twin()->isBoundary()) {
        queue.remove(h->edge()->record);
      }
      h = h->twin()->next();
    } while (h != v1->halfedge());

    // collapse best edge and assign position and quadric for new vertex
    VertexIter v = mesh.collapseEdge(best_edge);
    v->position = best_record.optimalPoint;
    v->quadric = new_quadric;

    // recompute and insert edge records of edges touching the new vertex into queue
    h = v->halfedge();
    do {
      if (!h->isBoundary() || !h->twin()->isBoundary()) {
        EdgeIter e = h->edge();
        e->record = EdgeRecord(e);
        queue.insert(e->record);
      }
      h = h->twin()->next();
    } while (h != v->halfedge());
  }
}

void MeshResampler::resample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute the mean edge length.
  // Repeat the four main steps for 5 or 6 iterations
  // -> Split edges much longer than the target length (being careful about
  //    how the loop is written!)
  // -> Collapse edges much shorter than the target length.  Here we need to
  //    be EXTRA careful about advancing the loop, because many edges may have
  //    been destroyed by a collapse (which ones?)
  // -> Now flip each edge if it improves vertex degree
  // -> Finally, apply some tangential smoothing to the vertex positions
  showError("resample() not implemented.");
}
}  // namespace CMU462
