#include <math.h>
#include <string>

#include "../common/macro.h"
#include "math.h"

using namespace math;

/* ----- vec3 ----- */

vec3::vec3() : x(0.f), y(0.f), z(0.f) {}
vec3::vec3(float x, float y, float z) : x(x), y(y), z(z) {}

void vec3::operator= (const vec3 & v2)
{
  DBG_VALID_VEC(v2);
  x = v2.x;
  y = v2.y;
  z = v2.z;
}
void vec3::operator+= (const vec3 & v2)
{
  DBG_VALID_VEC(v2);
  x += v2.x;
  y += v2.y;
  z += v2.z;
}
bool vec3::operator== (const vec3 & v2) const
{ 
  DBG_VALID_VEC(v2);
  return x == v2.x && y == v2.y && z == v2.z;
}
bool vec3::operator!= (const vec3 & v2) const
{ 
  DBG_VALID_VEC(v2);
  return !operator==(v2);
}

vec3 vec3::operator- (const vec3 & v2) const
{ 
  DBG_VALID_VEC(v2);
  return vec3(x - v2.x, y - v2.y, z - v2.z);
}
vec3 vec3::operator+ (const vec3 & v2) const
{ 
  DBG_VALID_VEC(v2);
  return vec3(x + v2.x, y + v2.y, z + v2.z);
}
vec3 vec3::operator*(float scalar) const
{ 
  DBG_VALID_FLOAT(scalar);
  return vec3(x*scalar, y*scalar, z*scalar);
}
vec3 vec3::operator*(const mat4 & m) const
{ 
  DBG_VALID_MAT4(m);
  mat vec(*this);
  vec = m * vec;
  return vec3(vec[0][0], vec[1][0], vec[2][0]);
}
vec3 vec3::operator/ (const float scalar) const
{ 
  DBG_VALID_FLOAT(scalar);
  return vec3(x / scalar, y / scalar, z / scalar);
}
vec3 vec3::operator- () const
{
  return vec3(-x, -y, -z);
}

float	vec3::length() const
{ 
  return sqrt(x*x + y*y + z*z);
}
float	vec3::unsqrt_length() const
{
  return (x*x + y*y + z*z); 
}
vec3 vec3::normalized() const
{
  DBG_ASSERT(length() != (0.f));
  return *this / length(); 
}

float vec3::dot(const vec3 & v1, const vec3 & v2)
{
  DBG_VALID_VEC(v1);
  DBG_VALID_VEC(v2);
  return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}
vec3 vec3::cross(const vec3 & v1, const vec3 & v2)
{
  DBG_VALID_VEC(v1);
  DBG_VALID_VEC(v2);
  float x = v1.y*v2.z - v2.y*v1.z;
  float y = v1.z*v2.x - v2.z*v1.x;
  float z = v1.x*v2.y - v2.x*v1.y;
  return vec3(x, y, z);
}

/* ----- mat4 ----- */

mat::mat() : data(nullptr), nbrOfRow(0), nbrOfCol(0)
{

}
mat::mat(int row, int col, float scalar) : mat()
{
  try
  {
    nbrOfRow = row;
    nbrOfCol = col;
    data = new float[nbrOfRow * nbrOfCol];
    for (int row = 0; row < nbrOfRow; row++)
      for (int col = 0; col < nbrOfCol; col++)
        if (row == col) (*this)[row][col] = scalar;
        else (*this)[row][col] = 0.f;
  }
  catch (const std::bad_alloc &)
  {
    nbrOfRow = 0;
    nbrOfCol = 0;
    SAFE_DELETE_TAB(data);
    data = nullptr;
  }
}
mat::mat(int row, int col) : mat(row, col, 1.f) {}
mat::mat(int row, int col, float tab[]) : mat(row, col)
{
  for (int n = 0; n < nbrOfRow * nbrOfCol; n++)
  {
    DBG_VALID_FLOAT(tab[n]);
    data[n] = tab[n];
  }
}
mat::mat(const vec3 & v) : mat(4, 1)
{
  data[0] = v.x;
  data[1] = v.y;
  data[2] = v.z;
  data[3] = 1.f;
}
mat::mat(const mat & m) : mat(m.nbrOfRow, m.nbrOfCol)
{
  for (int n = 0; n < nbrOfRow * nbrOfCol; n++) data[n] = m.data[n];
}
mat::~mat()
{
  SAFE_DELETE_TAB(data);
}

float * mat::operator[] (int row)
{
  return (data + (row * nbrOfCol));
}
float * mat::operator[] (int row) const
{
  return (data + (row * nbrOfCol));
}
mat  mat::operator*  (float scalar) const
{
  mat r(nbrOfRow, nbrOfCol);
  for (int n = 0; n < nbrOfRow * nbrOfCol; n++) r.data[n] = data[n] * scalar;
  return r;
}
mat  mat::operator/  (float scalar) const
{
  mat r(nbrOfRow, nbrOfCol);
  for (int n = 0; n < nbrOfRow * nbrOfCol; n++) r.data[n] = data[n] / scalar;
  return r;
}
void mat::operator*= (float scalar)
{
  for (int n = 0; n < nbrOfRow * nbrOfCol; n++) data[n] *= scalar;
}
void mat::operator/= (float scalar)
{
  for (int n = 0; n < nbrOfRow * nbrOfCol; n++) data[n] /= scalar;
}
mat mat::operator+  (const mat & m) const
{
  DBG_VALID_MAT(m);
  DBG_ASSERT(nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol);
  mat r(nbrOfRow, nbrOfCol);
  if (nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol)
  {
    for (int n = 0; n < nbrOfRow * nbrOfCol; n++) r.data[n] = data[n] + m.data[n];
    return r;
  }
}
mat mat::operator-  (const mat & m) const
{
  DBG_VALID_MAT(m);
  DBG_ASSERT(nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol);
  mat r(nbrOfRow, nbrOfCol);
  if (nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol)
  {
    for (int n = 0; n < nbrOfRow * nbrOfCol; n++) r.data[n] = data[n] - m.data[n];
    return r;
  }
}
void mat::operator+= (const mat & m) const
{
  DBG_VALID_MAT(m);
  DBG_ASSERT(nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol);
  if (nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol)
    for (int n = 0; n < nbrOfRow * nbrOfCol; n++) data[n] += m.data[n];
}
void mat::operator-= (const mat & m)
{
  DBG_VALID_MAT(m);
  DBG_ASSERT(nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol);
  if (nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol)
    for (int n = 0; n < nbrOfRow * nbrOfCol; n++) data[n] -= m.data[n];
}
mat mat::operator*  (const mat & m) const
{
  DBG_VALID_MAT(m);
  DBG_ASSERT(nbrOfCol == m.nbrOfRow);
  if (nbrOfCol == m.nbrOfRow)
  {
    mat r(nbrOfRow, m.nbrOfCol);
    for (int row = 0; row < nbrOfRow; row++)
    {
      for (int col = 0; col < m.nbrOfCol; col++)
      {
        float newElement = 0.f;
        for (int n = 0; n < nbrOfCol; n++)
        {
          float A = (*this)[row][n];
          float B = m[n][col];
          newElement += (*this)[row][n] * m[n][col];
        }
        r[row][col] = newElement;
      }
    }
    return r;
  }
}
void mat::operator*= (const mat & m)
{
  DBG_VALID_MAT(m);
  DBG_ASSERT(nbrOfCol == m.nbrOfRow);
  if (nbrOfCol == m.nbrOfRow)
  {
    (*this) = (*this) * m;
  }
}
mat & mat::operator=  (const mat & m)
{
  DBG_VALID_MAT(m);
  SAFE_DELETE_TAB(data);

  nbrOfRow = m.nbrOfRow;
  nbrOfCol = m.nbrOfCol;
  data = new float[nbrOfRow * nbrOfCol];

  for (int n = 0; n < nbrOfRow * nbrOfCol; n++) data[n] = m.data[n];

  return *this;
}
bool mat::operator== (const mat & m)
{
  DBG_VALID_MAT(m);
  DBG_ASSERT(nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol);
  if (nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol)
  {
    for (int n = 0; n < nbrOfRow * nbrOfCol; n++) if (data[n] <= m.data[n] - FLOATING_POINT_ERROR && data[n] >= m.data[n] + FLOATING_POINT_ERROR) return false;
    return true;
  }
  return false;
}
bool mat::operator!= (const mat & m)
{
  DBG_VALID_MAT(m);
  DBG_ASSERT(nbrOfRow == m.nbrOfRow && nbrOfCol == m.nbrOfCol);
  return !((*this) == m);
}

mat mat::inverse() const
{
  if (nbrOfRow == nbrOfCol)
  {
    mat tmp(*this);
    mat res(nbrOfRow, nbrOfCol);

    // Gauss jordan algorithm
    for (int n = 0; n < nbrOfRow; n++)
    {
      if (tmp[n][n] != 1.f && tmp[n][n] != 0.f) // rounding error?
      {
        float ratio = 1 / tmp[n][n];
        tmp.multiplyRow(n, ratio);
        res.multiplyRow(n, ratio);
      }
      for (int row = 0; row < nbrOfRow; row++)
      {
        if (row != n)
        {
          res.multipleRowSubstraction(row, n, tmp[row][n]);
          tmp.multipleRowSubstraction(row, n, tmp[row][n]);
        }
      }
    }

    return res;
  }
  else
    return pinverse();
}
mat mat::inverse(const mat & m)
{
  DBG_VALID_MAT(m);
  return m.inverse();
}

mat mat::transpose() const
{
  mat r(nbrOfCol, nbrOfRow);
  for (int row = 0; row < nbrOfRow; row++)
  {
    for (int col = 0; col < nbrOfCol; col++)
    {
      r[col][row] = data[row * nbrOfCol + col];
    }
  }
  return r;
}
mat mat::transpose(const mat & m)
{
  DBG_VALID_MAT(m);
  return m.transpose();
}

void mat::multiplyRow(int row, float scalar)
{
  for (int n = row * nbrOfCol; n < (row + 1) * nbrOfCol; n++) data[n] *= scalar;
}
void mat::swapRows(int row1, int row2)
{
  int row1p = row1 * nbrOfCol;
  int row2p = row2 * nbrOfCol;
  for (int n = 0; n < nbrOfCol; n++)
  {
    float tmp = data[row1p + n];
    data[row1p + n] = data[row2p + n];
    data[row2p + n] = tmp;
  }
}
void mat::addRows(int row1, int row2) // row1 = row1 + row2
{
  int row1p = row1 * nbrOfCol;
  int row2p = row2 * nbrOfCol;
  for (int n = 0; n < nbrOfCol; n++) data[row1p + n] += data[row2p + n];
}
void mat::multipleRowAddition(int row1, int row2, float nbrOfAdditions)
{
  int row1p = row1 * nbrOfCol;
  int row2p = row2 * nbrOfCol;
  for (int n = 0; n < nbrOfCol; n++) data[row1p + n] += nbrOfAdditions * data[row2p + n];
}
void mat::substractRows(int row1, int row2) // row1 = row1 - row2
{
  int row1p = row1 * nbrOfCol;
  int row2p = row2 * nbrOfCol;
  for (int n = 0; n < nbrOfCol; n++) data[row1p + n] -= data[row2p + n];
}
void mat::multipleRowSubstraction(int row1, int row2, float nbrOfSubstractions)
{
  int row1p = row1 * nbrOfCol;
  int row2p = row2 * nbrOfCol;
  for (int n = 0; n < nbrOfCol; n++) data[row1p + n] -= nbrOfSubstractions * data[row2p + n];
}

mat4::mat4() : mat(4, 4) {}
mat4::mat4(mat & m) : mat4()
{
  DBG_ASSERT(m.nbrOfRow == 4 && m.nbrOfCol == 4);
  if (m.nbrOfRow == 4 && m.nbrOfCol == 4)
  {
    for (int n = 0; n < 16; n++)
    {
      data[n] = m.data[n];
    }
  }
}
mat4::mat4(float tab[]) : mat(4, 4, tab) {}
mat4::mat4(float scalar) : mat(4, 4, scalar) {}

void mat4::operator=(const mat & m)
{
  DBG_ASSERT(m.nbrOfRow == 4 && m.nbrOfCol == 4);
  if (m.nbrOfRow == 4 && m.nbrOfCol == 4)
  {
    for (int n = 0; n < 16; n++)
    {
      data[n] = m.data[n];
    }
  }
}
mat  mat4::operator+  (const mat & m) const
{
  return mat::operator+(m);
}
mat  mat4::operator-  (const mat & m) const
{
  return mat::operator-(m);
}
void mat4::operator+= (const mat & m) const
{
  mat::operator+=(m);
}
void mat4::operator-= (const mat & m)
{
  mat::operator-=(m);
}
mat  mat4::operator*  (const mat & m) const
{
  return mat::operator*(m);
}
void mat4::operator*= (const mat & m)
{
  mat::operator*=(m);
}
bool mat4::operator== (const mat & m)
{
  return mat::operator==(m);
}
bool mat4::operator!= (const mat & m)
{
  return mat::operator!=(m);
}

vec3 mat4::operator*(const vec3 & v)
{
  DBG_VALID_VEC(v);
  float x = (data[0] * v.x + data[1] * v.y + data[2] * v.z) + data[3];
  float y = (data[4] * v.x + data[5] * v.y + data[6] * v.z) + data[7];
  float z = (data[8] * v.x + data[9] * v.y + data[10] * v.z) + data[11];
  return{ x, y, z };
}

mat4 mat4::R_Transform(float r_angle, const math::vec3 & r_axis)
{
  DBG_VALID_VEC(r_axis);
  DBG_VALID_FLOAT(r_angle);
  float sin_angle = sin(r_angle);
  float cos_angle = cos(r_angle);
  float inv_cos_angle = 1.f - cos_angle;

  math::vec3 axis = r_axis.normalized();

  mat4 r(1);
  r.data[0] = inv_cos_angle * axis.x * axis.x + cos_angle;
  r.data[1] = inv_cos_angle * axis.x * axis.y - (sin_angle * axis.z);
  r.data[2] = inv_cos_angle * axis.x * axis.z + (sin_angle * axis.y);
  r.data[4] = inv_cos_angle * axis.y * axis.x + (sin_angle * axis.z);
  r.data[5] = inv_cos_angle * axis.y * axis.y + cos_angle;
  r.data[6] = inv_cos_angle * axis.y * axis.z - (sin_angle * axis.x);
  r.data[8] = inv_cos_angle * axis.z * axis.x - (sin_angle * axis.y);
  r.data[9] = inv_cos_angle * axis.z * axis.y + (sin_angle * axis.x);
  r.data[10] = inv_cos_angle * axis.z * axis.z + cos_angle;
  return r;
}
mat4 mat4::R_Transform(const quat & q)
{
  DBG_VALID_QUAT(q);

  float angle = acos(q.w) * 2.f;
  float tmp = sin(angle / 2.f);
  vec3 axis = (vec3(q.x, q.y, q.z) / tmp).normalized();

  // Page 466, Graphics Gems

  float sin_angle = sin(angle);
  float cos_angle = cos(angle);
  float inv_cos_angle = 1.f - cos_angle;

  mat4 r(1);
  r.data[0] = inv_cos_angle * axis.x * axis.x + cos_angle;
  r.data[1] = inv_cos_angle * axis.x * axis.y - (sin_angle * axis.z);
  r.data[2] = inv_cos_angle * axis.x * axis.z + (sin_angle * axis.y);
  r.data[4] = inv_cos_angle * axis.y * axis.x + (sin_angle * axis.z);
  r.data[5] = inv_cos_angle * axis.y * axis.y + cos_angle;
  r.data[6] = inv_cos_angle * axis.y * axis.z - (sin_angle * axis.x);
  r.data[8] = inv_cos_angle * axis.z * axis.x - (sin_angle * axis.y);
  r.data[9] = inv_cos_angle * axis.z * axis.y + (sin_angle * axis.x);
  r.data[10] = inv_cos_angle * axis.z * axis.z + cos_angle;
  return r;
}
mat4 mat4::T_Transform(const vec3 & t)
{
  DBG_VALID_VEC(t);
  mat4 res(1.f);
  res[0][3] = t.x;
  res[1][3] = t.y;
  res[2][3] = t.z;
  return res;
}
mat4 mat4::S_Transform(const vec3 & s)
{
  DBG_VALID_VEC(s);
  mat4 res(1.f);
  res[0][0] = s.x;
  res[1][1] = s.y;
  res[2][2] = s.z;
  return res;
}

vec3 math::operator*(const mat4 & m, const vec3 & v) { return v * m; }

/* ----- quat ----- */

quat::quat() : w(0.f), x(0.f), y(0.f), z(0.f) {}
quat::quat(float f1, float f2, float f3, float f4) : w(f1), x(f2), y(f3), z(f4) {}
quat::quat(float t[4]) : w(t[0]), x(t[1]), y(t[2]), z(t[3]) {}
quat::quat(float f, math::vec3 v) : w(f), x(v.x), y(v.y), z(v.z) {}

quat quat::operator-() const
{
  return quat(w, -x, -y, -z);
}
quat quat::operator*(float s) const
{
  DBG_VALID_FLOAT(s);
  return quat(w*s,x*s,y*z,z*s);
}
quat quat::operator*(const quat & q2) const
{
  DBG_VALID_QUAT(q2);
  vec3 v1 = { x, y, z };
  vec3 v2 = { q2.x, q2.y, q2.z };
  float w1 = w;
  float w2 = q2.w;
  vec3 vf = v1 * w2 + v2 * w1 + vec3::cross(v1, v2);
  float wf = w1 * w2 - vec3::dot(v1, v2);
  return quat(wf, vf.x, vf.y, vf.z);
}
quat quat::operator+(const quat & q2) const
{
  DBG_VALID_QUAT(q2);
  return quat(w + q2.w ,x + q2.x ,y + q2.y ,z + q2.z);
}

bool quat::operator==(const quat & q2) const
{
  DBG_VALID_QUAT(q2);
  return this->x == q2.x && this->y == q2.y && this->z == q2.z && this->w == q2.w;
}
bool quat::operator!=(const quat & q2) const
{
  return !(*this == q2);
}

quat quat::R_quat(float radian, const vec3 & axis)
{
  DBG_VALID_FLOAT(radian);
  DBG_VALID_VEC(axis);
  vec3 n_axis = axis.normalized();
  float tmp = sin(radian / 2);
  float w = cos(radian / 2);
  return quat(w, n_axis * tmp);
}
quat quat::extractQuat(const mat4 & m)
{
  quat q;
  float trace = m.data[0] + m.data[5] + m.data[10];
  if (trace > 0) {
    float s = 0.5f / sqrtf(trace + 1.0f);
    q.w = 0.25f / s;
    q.x = (m.data[9] - m.data[6]) * s;
    q.y = (m.data[2] - m.data[8]) * s;
    q.z = (m.data[4] - m.data[1]) * s;
  }
  else {
    if (m.data[0] > m.data[5] && m.data[0] > m.data[10]) {
      float s = 2.0f * sqrtf(1.0f + m.data[0] - m.data[5] - m.data[10]);
      q.w = (m.data[9] - m.data[6]) / s;
      q.x = 0.25f * s;
      q.y = (m.data[1] + m.data[4]) / s;
      q.z = (m.data[2] + m.data[8]) / s;
    }
    else if (m.data[5] > m.data[10]) {
      float s = 2.0f * sqrtf(1.0f + m.data[5] - m.data[0] - m.data[10]);
      q.w = (m.data[2] - m.data[8]) / s;
      q.x = (m.data[1] + m.data[4]) / s;
      q.y = 0.25f * s;
      q.z = (m.data[6] + m.data[9]) / s;
    }
    else {
      float s = 2.0f * sqrtf(1.0f + m.data[10] - m.data[0] - m.data[5]);
      q.w = (m.data[4] - m.data[1]) / s;
      q.x = (m.data[2] + m.data[8]) / s;
      q.y = (m.data[6] + m.data[9]) / s;
      q.z = 0.25f * s;
    }
  }
  return q;
}
quat quat::slerp(const quat & q1, const quat & q2, float alpha)
{
  DBG_VALID_FLOAT(alpha);
  DBG_VALID_QUAT(q1);
  DBG_VALID_QUAT(q2);
  float angle = (acos(q2.w) * 2 - acos(q1.w) * 2);
  float tmp0 = (sin(1 - alpha)*angle) / sin(angle);
  float tmp1 = (sin(angle) * alpha) / sinh(angle);
  return q1*tmp0 + q2*tmp1;
}

mat math::triangleProjectionMatrix(const math::vec3 & x1, const math::vec3 & x2, const math::vec3 & x3)
{
  DBG_VALID_VEC(x1);
  DBG_VALID_VEC(x2);
  DBG_VALID_VEC(x3);
  math::vec3 tmp1 = x2 - x1;
  math::vec3 tmp2 = x3 - x1;
  math::vec3 tmp3 = math::vec3::cross(tmp1, tmp2);
  math::vec3 planeNormal = math::vec3::cross(x2 - x1, x3 - x1).normalized();
  math::vec3 Ox = (x2 - x1).normalized();
  math::vec3 Oy = math::vec3::cross(planeNormal, Ox).normalized();
  DBG_VALID_VEC(Ox);
  DBG_VALID_VEC(Oy);
  math::mat projectionMatrix(2, 3);
  projectionMatrix[0][0] = Ox.x;
  projectionMatrix[0][1] = Oy.x;
  projectionMatrix[0][2] = Ox.y;
  projectionMatrix[1][0] = Oy.y;
  projectionMatrix[1][1] = Ox.z;
  projectionMatrix[1][2] = Oy.z;
  return projectionMatrix;
}