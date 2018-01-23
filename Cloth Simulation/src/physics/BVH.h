#ifndef __H__BVH__
#define __H__BVH__

#include <vector>
#include <iostream>

#include "../common/constants.h"
#include "../common/macro.h"
#include "../maths/math.h"
#include "cloth.h"

template <size_t n>
struct CollisionData
{
  math::vec3 p[n];
  math::vec3 v[n];
  float m[n];
  size_t i[n]; // lms indices
};

struct CollisionData_Triangle : CollisionData<3>
{
  CollisionData_Triangle();
  CollisionData_Triangle(size_t i1, size_t i2, size_t i3, float m1, float m2, float m3);
  void updateData(LinearMotionSystem & _lms);
};
struct CollisionData_Edge : CollisionData<2>
{
  CollisionData_Edge();
  CollisionData_Edge(size_t i1, size_t i2, float m1, float m2);
  void updateData(LinearMotionSystem & _lms);
};
struct CollisionData_Point : CollisionData<1>
{
  CollisionData_Point();
  CollisionData_Point(size_t lms_indice, float mass);
  CollisionData_Point(size_t lms_indice, float mass, const math::vec3 & position, const math::vec3 & velocity);
  void updateData(LinearMotionSystem & _lms);
};

struct AABB
{
  math::vec3 center;
  math::vec3 radius;
  AABB * left = nullptr;
  AABB * right = nullptr;
  std::vector<AABB*> parents;

  AABB(const math::vec3 & p1, const math::vec3 & p2, const math::vec3 & p3);
  AABB(AABB & b1, AABB & b2);
  AABB(const math::vec3 p[], size_t nbrOfPoints);
  ~AABB();
  std::vector<AABB*> collision(const AABB & aabb);
  void update(bool updateChildren = true);
  void update(const math::vec3 & p1, const math::vec3 & p2, const math::vec3 & p3);
  void updateParents();
};

struct ClothCollisionModel
{
  std::vector<CollisionData_Triangle> triangles;
  std::vector<AABB*> triangleBoxes;
  AABB * root;
  LinearMotionSystem & _lms;
  float thickness;
  float stiffness = 1.f;

  gl::GLuint VAO = NULL;
  gl::GLuint VBO = NULL;
  gl::GLuint program;
  std::vector<gl::GLfloat> toDraw;

  int nbrOfCollisions = 0;

  ClothCollisionModel(LinearMotionSystem & lms);
  ~ClothCollisionModel();

  void setStiffess(float _stiffness);
  void init(Cloth & c);
  void initGL(gl::GLuint _program);
  void render(const math::mat & projMatrix);

  void updateAllTriangleData();
  void ClothCollisionModel::updateTriangleData(size_t i, bool updateAABB = true, bool updateParents = true);
  void resolveInternalCollisions();

  void resolveTriangleTriangleCollision(LinearMotionSystem & _lms, CollisionData_Triangle & t1, CollisionData_Triangle & t2);
  bool resolvePointTriangleCollision(LinearMotionSystem & _lms, CollisionData_Point & p, CollisionData_Triangle & t, float threshold, float stiffnessCoefficient);
};

#endif //__H__BVH__