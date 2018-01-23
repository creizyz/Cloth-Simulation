#include "BVH.h"

/* COLLISION DATA */

CollisionData_Triangle::CollisionData_Triangle()
{ }
CollisionData_Triangle::CollisionData_Triangle(size_t i1, size_t i2, size_t i3, float m1, float m2, float m3)
{
  i[0] = i1;
  i[1] = i2;
  i[2] = i3;
  m[0] = m1;
  m[1] = m2;
  m[2] = m3;
}
void CollisionData_Triangle::updateData(LinearMotionSystem & _lms)
{
  for (size_t n = 0; n < 3; n++)
  {
    p[n] = _lms.get_linear_position(i[n]);
    v[n] = _lms.get_linear_velocity(i[n]);
  }
}
CollisionData_Edge::CollisionData_Edge()
{ }
CollisionData_Edge::CollisionData_Edge(size_t i1, size_t i2, float m1, float m2)
{
  i[0] = i1;
  i[1] = i2;
  m[0] = m1;
  m[1] = m2;
}
void CollisionData_Edge::updateData(LinearMotionSystem & _lms)
{
  for (size_t n = 0; n < 2; n++)
  {
    p[n] = _lms.get_linear_position(i[n]);
    v[n] = _lms.get_linear_velocity(i[n]);
  }
}
CollisionData_Point::CollisionData_Point() {}
CollisionData_Point::CollisionData_Point(size_t lms_indice, float mass)
{
  i[0] = lms_indice;
  m[0] = mass;
}
CollisionData_Point::CollisionData_Point(size_t lms_indice, float mass, const math::vec3 & position, const math::vec3 & velocity)
{
  i[0] = lms_indice;
  m[0] = mass;
  p[0] = position;
  v[0] = velocity;
}
void CollisionData_Point::updateData(LinearMotionSystem & _lms)
{
  p[0] = _lms.get_linear_position(i[0]);
  v[0] = _lms.get_linear_velocity(i[0]);
}

/* AABB */

AABB::AABB(const math::vec3 & p1, const math::vec3 & p2, const math::vec3 & p3) :
  radius(0.f, 0.f, 0.f)
{
  center = (p1 + p2 + p3) / 3.f;
  math::vec3 vec[] = { p1 - center, p2 - center, p3 - center };
  for (size_t n = 0; n < 3; n++)
  {
    if (vec[n].x > radius.x) radius.x = vec[n].x;
    if (vec[n].y > radius.y) radius.y = vec[n].y;
    if (vec[n].z > radius.z) radius.z = vec[n].z;
  }
}
AABB::AABB(AABB & b1, AABB & b2) :
  radius(0.f, 0.f, 0.f)
{
  center = (b1.center + b2.center) / 2.f;
  math::vec3 relRadius[] = {
    b1.radius + (b1.center - center),
    b2.radius + (b2.center - center)
  };
  for (size_t n = 0; n < 2; n++)
  {
    if (relRadius[n].x > radius.x) radius.x = relRadius[n].x;
    if (relRadius[n].y > radius.y) radius.y = relRadius[n].y;
    if (relRadius[n].z > radius.z) radius.z = relRadius[n].z;
  }
  left = &b1;
  right = &b2;
  b1.parents.push_back(this);
  b2.parents.push_back(this);
}
AABB::AABB(const math::vec3 p[], size_t nbrOfPoints) :
  center(0.f, 0.f, 0.f),
  radius(0.f, 0.f, 0.f)
{
  for (size_t n = 0; n < nbrOfPoints; n++) center += p[n];
  center = center / (float)nbrOfPoints;
  for (size_t n = 0; n < nbrOfPoints; n++)
  {
    math::vec3 relVec = p[n] - center;
    if (relVec.x > radius.x) radius.x = relVec.x;
    if (relVec.y > radius.y) radius.y = relVec.y;
    if (relVec.z > radius.z) radius.z = relVec.z;
  }
}
AABB::~AABB()
{
  SAFE_DELETE(left);
  SAFE_DELETE(right);
  for (size_t n = 0; n < parents.size(); n++)
  {
    if (parents[n]->left == this) parents[n]->left = nullptr;
    if (parents[n]->right == this) parents[n]->right = nullptr;
  }
}
std::vector<AABB*> AABB::collision(const AABB & aabb)
{
  std::vector<AABB*> collided;
  math::vec3 dst = center - aabb.center;
  if (abs(dst.x) <= (radius.x + aabb.radius.x) &&
    abs(dst.y) <= (radius.y + aabb.radius.y) &&
    abs(dst.z) <= (radius.z + aabb.radius.z))
  {
    if (left == nullptr && right == nullptr && &aabb != this)
    {
      collided.push_back(this);
    }
    else
    {
      if (left != nullptr)
      {
        std::vector<AABB*> childCollision = left->collision(aabb);
        if (childCollision.size() > 0)
        {
          for (size_t n = 0; n < childCollision.size(); n++)
          {
            bool present = false;
            for (size_t i = 0; i < collided.size(); i++)
            {
              if (collided[i] == childCollision[n])
              {
                present = true;
                break;
              }
            }
            if (!present) collided.push_back(childCollision[n]);
          }
        }
      }
      if (right != nullptr) // <- still unfound
      {
        std::vector<AABB*> childCollision = right->collision(aabb);
        if (childCollision.size() > 0)
        {
          for (size_t n = 0; n < childCollision.size(); n++)
          {
            bool present = false;
            for (size_t i = 0; i < collided.size(); i++)
            {
              if (collided[i] == childCollision[n])
              {
                present = true;
                break;
              }
            }
            if (!present) collided.push_back(childCollision[n]);
          }
        }
      }
    }
  }
  return collided;
}
void AABB::update(bool updateChildren)
{
  bool hasLeftChild(left != nullptr);
  bool hasRightChild(right != nullptr);
  if (hasLeftChild || hasRightChild)
  {
    center = math::vec3(0.f, 0.f, 0.f);
    radius = math::vec3(0.f, 0.f, 0.f);

    float count = 0.f;
    if (hasLeftChild)
    {
      if (updateChildren) left->update(); // <- first thing we update the children
      count++;
      center += left->center;
    }
    if (hasRightChild)
    {
      if (updateChildren) right->update(); // <- first thing we update the children
      count++;
      center += right->center;
    }
    center = center / count;

    if (hasLeftChild)
    {
      math::vec3 relRadius = left->radius + (left->center - center);
      if (relRadius.x > radius.x) radius.x = relRadius.x;
      if (relRadius.y > radius.y) radius.y = relRadius.y;
      if (relRadius.z > radius.z) radius.z = relRadius.z;
    }
    if (hasRightChild)
    {
      math::vec3 relRadius = right->radius + (right->center - center);
      if (relRadius.x > radius.x) radius.x = relRadius.x;
      if (relRadius.y > radius.y) radius.y = relRadius.y;
      if (relRadius.z > radius.z) radius.z = relRadius.z;
    }
  }
}
void AABB::update(const math::vec3 & p1, const math::vec3 & p2, const math::vec3 & p3)
{
  center = (p1 + p2 + p3) / 3.f;
  math::vec3 vec[] = { p1 - center, p2 - center, p3 - center };
  radius = math::vec3(0.f, 0.f, 0.f);
  for (size_t n = 0; n < 3; n++)
  {
    if (vec[n].x > radius.x) radius.x = vec[n].x;
    if (vec[n].y > radius.y) radius.y = vec[n].y;
    if (vec[n].z > radius.z) radius.z = vec[n].z;
  }
}
void AABB::updateParents()
{
  for (size_t n = 0; n < parents.size(); n++)
  {
    parents[n]->update(false);
    parents[n]->updateParents();
  }
}

/* CLOTH MODEL */

ClothCollisionModel::ClothCollisionModel(LinearMotionSystem & lms) : _lms(lms), root(nullptr) {}
ClothCollisionModel::~ClothCollisionModel()
{
  SAFE_DELETE(root);
  if (VAO != NULL) gl::glDeleteVertexArrays(1, &VAO);
  if (VBO != NULL) gl::glDeleteBuffers(1, &VBO);
}
void ClothCollisionModel::setStiffess(float _stiffness)
{
  stiffness = _stiffness;
}
void ClothCollisionModel::init(Cloth & c)
{
  thickness = c.thickness;
  triangles.resize(c.nbrOfTriangles);
  std::vector<AABB*> aabbs_lvl_inf;
  for (size_t n = 0; n < c.nbrOfTriangles; n++)
  {
    size_t iT[]{ c.triangles[3 * n], c.triangles[3 * n + 1], c.triangles[3 * n + 2] };
    triangles[n] = CollisionData_Triangle(c.posCur[iT[0]], c.posCur[iT[1]], c.posCur[iT[2]], c.mass[iT[0]], c.mass[iT[1]], c.mass[iT[2]]);
    updateTriangleData(n, false);
    // Create our first level of AABBs
    aabbs_lvl_inf.push_back(new AABB(triangles[n].p[0], triangles[n].p[1], triangles[n].p[2]));
    aabbs_lvl_inf.back()->radius += math::vec3(1.f, 1.f, 1.f) * c.thickness;
  }
  triangleBoxes = aabbs_lvl_inf;

  // In the second level, we have to pair up the triangles depending on their topology (zigzags)
  std::vector<AABB*> aabbs_lvl_sup;
  int currentLine = 0;
  int i = 0;
  int step = 2;

  while (i < triangles.size() && i + (step / 2) < triangles.size())
  {
    aabbs_lvl_sup.push_back(new AABB(*aabbs_lvl_inf[i], *aabbs_lvl_inf[i + (step / 2)]));
    if ((i + step) / c.width == currentLine) i += step; // <- going forward does not change the line?
    else
    {
      if (c.width % 2 != 0 && i + c.width < triangles.size()) aabbs_lvl_sup.push_back(new AABB(*aabbs_lvl_inf[i + (step / 2)], *aabbs_lvl_inf[i + (step / 2) + c.width])); // odd number of triangles in a row
      i += c.width;
      step *= -1;
    }
  }

  if (triangles.size() % 2 != 0) // if there is an off number of triangles, we have a triangle that was left alone and we have to add it to this level
  {
    if (currentLine % 2 == 0) aabbs_lvl_sup.push_back(aabbs_lvl_inf.back()); // last element of the last line if we are on an even line
    else aabbs_lvl_sup.push_back(aabbs_lvl_inf[triangles.size() - c.width + 1]); // first element of the last line if we are on an odd line
  }

  // Now we have ordoned our bounding boxes by topology, creating higher levels of the bounding hierarchy is easier
  aabbs_lvl_inf = aabbs_lvl_sup;
  aabbs_lvl_sup.clear();

  while (aabbs_lvl_inf.size() > 1)
  {
    for (size_t n = 0; n + 1 < aabbs_lvl_inf.size(); n += 2)
      aabbs_lvl_sup.push_back(new AABB(*aabbs_lvl_inf[n], *aabbs_lvl_inf[n + 1]));
    if (aabbs_lvl_inf.size() % 2 != 0) aabbs_lvl_sup.push_back(aabbs_lvl_inf.back());
    aabbs_lvl_inf = aabbs_lvl_sup;
    aabbs_lvl_sup.clear();
  }
  root = aabbs_lvl_inf.back();
  int n = 0;
}
void ClothCollisionModel::initGL(gl::GLuint _program)
{
  toDraw.clear();
  program = _program;
  gl::glGenVertexArrays(1, &VAO);
  gl::glGenBuffers(1, &VBO);

  gl::glBindVertexArray(VAO);
  gl::glBindBuffer(gl::GL_ARRAY_BUFFER, VBO);

  gl::GLint position_attribute2 = gl::glGetAttribLocation(program, "position");
  gl::glVertexAttribPointer(position_attribute2, 3, gl::GL_FLOAT, gl::GL_FALSE, 0, 0);
  gl::glEnableVertexAttribArray(position_attribute2);
  gl::glBindVertexArray(0);
}
void ClothCollisionModel::render(const math::mat & projMatrix)
{
  if (toDraw.size() > 6)
  {
    gl::glUseProgram(program);

    gl::GLint matrixUniform = gl::glGetUniformLocation(program, "VPMatrix");
    if (matrixUniform != -1) gl::glUniformMatrix4fv(matrixUniform, 1, gl::GL_FALSE, projMatrix.data);
    gl::GLint colorUniform = gl::glGetUniformLocation(program, "color");
    if (colorUniform != -1) gl::glUniform4f(colorUniform, 1.f, .5f, 0.f, 1.f);

    gl::glBindVertexArray(VAO);
    gl::glBindBuffer(gl::GL_ARRAY_BUFFER, VBO);
    gl::glBufferData(gl::GL_ARRAY_BUFFER, toDraw.size() * sizeof(gl::GLfloat), &toDraw[0], gl::GL_DYNAMIC_DRAW);
    gl::glDrawArrays(gl::GL_LINES, 0, toDraw.size() / 3);
    gl::glBindBuffer(gl::GL_ARRAY_BUFFER, 0);
    gl::glBindVertexArray(0);
  }
}
void ClothCollisionModel::updateAllTriangleData()
{
  for (size_t n = 0; n < triangles.size(); n++)
  {
    updateTriangleData(n, true, false);
  }
  root->update();
}
void ClothCollisionModel::updateTriangleData(size_t i, bool updateAABB, bool updateParents)
{
  DBG_ASSERT(i < triangles.size());
  if (i < triangles.size())
  {
    CollisionData_Triangle & t = triangles[i];
    for (size_t n = 0; n < 3; n++)
    {
      t.p[n] = _lms.get_linear_position(t.i[n]);
      t.v[n] = _lms.get_linear_velocity(t.i[n]);
    }
    if (updateAABB)
    {
      triangleBoxes[i]->update(triangles[i].p[0], triangles[i].p[1], triangles[i].p[2]);
      if (updateParents) triangleBoxes[i]->updateParents();
    }
  }
}
void ClothCollisionModel::resolveInternalCollisions()
{
  toDraw.clear();
  updateAllTriangleData();
  for (size_t i = 0; i < triangles.size(); i++)
  {
    CollisionData_Triangle & t1 = triangles[i];
    std::vector<AABB*> collided = root->collision(*triangleBoxes[i]);
    std::vector<size_t> collision_checked;
    for (size_t n = 0; n < collided.size(); n++)
    {
      int indexOfT2 = -1;
      for (size_t j = 0; j < triangles.size(); j++)
        if (triangleBoxes[j] == collided[n])
        {
          indexOfT2 = j;
          break;
        }
      if (indexOfT2 != -1)
      {
        bool checked = false;
        for (size_t k = 0; k < collision_checked.size(); k++) if (collision_checked[k] == indexOfT2) { checked = true; break; }
        if (!checked)
        {
          collision_checked.push_back(indexOfT2);
          CollisionData_Triangle & t2 = triangles[indexOfT2];
          resolveTriangleTriangleCollision(_lms, t1, t2);
          updateTriangleData(i);
          updateTriangleData(indexOfT2);
        }
      }
    }
  }
  //std::cout << "collisions: " << nbrOfCollisions << std::endl;
  nbrOfCollisions = 0;
}
void ClothCollisionModel::resolveTriangleTriangleCollision(LinearMotionSystem & _lms, CollisionData_Triangle & t1, CollisionData_Triangle & t2)
{
  for (size_t n = 0; n < 3; n++)
  {
    if (t1.i[n] != t2.i[0] && t1.i[n] != t2.i[1] && t1.i[n] != t2.i[2]) // <- if the point n of t1 is not also part of the CollisionData_Triangle t2 we check it against t2
    {
      size_t i = t1.i[n];
      resolvePointTriangleCollision(_lms, CollisionData_Point(t1.i[n], t1.m[n], t1.p[n], t1.v[n]), t2, thickness, stiffness);
    }
  }
}
bool ClothCollisionModel::resolvePointTriangleCollision(LinearMotionSystem & _lms, CollisionData_Point & p, CollisionData_Triangle & t, float threshold, float stiffnessCoefficient)
{
  math::vec3 side[]{ t.p[1] - t.p[0], t.p[2] - t.p[0] }; // triangle sides
  math::vec3 normal = math::vec3::cross(side[0], side[1]).normalized();
  float dst = math::vec3::dot(p.p[0] - t.p[2], normal);
  if (dst < threshold) // Possible collision
  {
    math::vec3 projP = p.p[0] - (normal * dst);
    math::vec3 vecP = projP - t.p[0];

    float dot00 = math::vec3::dot(side[0], side[0]); // <- can reuse it for every test on this triangle
    float dot01 = math::vec3::dot(side[0], side[1]);
    float dot11 = math::vec3::dot(side[1], side[1]);
    float invDenom = 1.f / ((dot00 * dot11) - (dot01 * dot01));

    float dotP0 = math::vec3::dot(vecP, side[0]); // <- calculate barycentric coordinates
    float dotP1 = math::vec3::dot(vecP, side[1]);
    float w[3]{
      0.f,
      ((dot11 * dotP0) - (dot01 * dotP1)) * invDenom,
      ((dot00 * dotP1) - (dot01 * dotP0)) * invDenom
    };
    w[0] = 1.f - w[1] - w[2];

    bool insideTriangle(true); // <- test if we are in the triangle +- the threshold
    for (int n = 0; n < 3; n++)
    {
      float relThreshold = threshold / t.p[n].length();
      if (w[n] < -relThreshold || w[n] > 1 + relThreshold)
      {
        insideTriangle = false;
        break;
      }
    }

    if (insideTriangle) // <- There is a collision
    {
      float length_vTN(0.f);
      for (size_t n = 0; n < 3; n++) length_vTN += math::vec3::dot(t.v[n] * w[n], normal);
      math::vec3 vTN = length_vTN * normal;
      math::vec3 vPN = math::vec3::dot(p.v[0], normal) * normal;
      math::vec3 vN_rel = vPN - vTN;

      float mT = t.m[0] * w[0] + t.m[1] * w[1] + t.m[2] * w[2];
      math::vec3 Ic_t(0.f, 0.f, 0.f), Ic_p(0.f, 0.f, 0.f);

      if (math::vec3::dot(vN_rel, normal) <= FLOATING_ERROR_COUNTERING) // <- if the point is moving toward the triangle we stop the relative movement
      {
        Ic_t = 0.5f * mT * vN_rel; // <- Counter relative velocity
        Ic_p = 0.5f * p.m[0] * -vN_rel; // <- p.m : mass of the point
        vN_rel = math::vec3(0.f, 0.f, 0.f);

        float o = std::max(threshold - (dst * 0.5f), 0.f); // <- overlap
        float Vr_length_max = o * LIMIT_COLLISION_PUSH_APART_FACTOR * INV_PHYSICS_TIME_STEP - vN_rel.length();
        float Ir_t_length_max = mT * Vr_length_max;
        float Ir_p_length_max = p.m[0] * Vr_length_max;

        float Ir_length = stiffnessCoefficient * o * PHYSICS_TIME_STEP;
        math::vec3 Ir_t = -std::min(Ir_t_length_max, Ir_length) * normal;
        math::vec3 Ir_p = std::min(Ir_p_length_max, Ir_length) * normal;

        for (size_t n = 0; n < 3; n++) _lms.apply_impulse(t.i[n], w[n] * (Ic_t + Ir_t));
        _lms.apply_impulse(p.i[0], Ic_p + Ir_p);
        nbrOfCollisions++;
        toDraw.push_back(p.p[0].x);
        toDraw.push_back(p.p[0].y);
        toDraw.push_back(p.p[0].z);
        math::vec3 pAfter = p.p[0] + (o * stiffnessCoefficient * normal);
        toDraw.push_back(pAfter.x);
        toDraw.push_back(pAfter.y);
        toDraw.push_back(pAfter.z);

        toDraw.push_back(projP.x);
        toDraw.push_back(projP.y);
        toDraw.push_back(projP.z);
        math::vec3 ppAfter = projP - (o * stiffnessCoefficient * normal);
        toDraw.push_back(ppAfter.x);
        toDraw.push_back(ppAfter.y);
        toDraw.push_back(ppAfter.z);
        return true;
      }
    }
  }
  return false;
}