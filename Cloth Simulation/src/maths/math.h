#ifndef __H__MATH__
#define __H__MATH__

#ifndef MATH_PI
#define MATH_PI 3.1415f
#endif

#ifndef FLOATING_POINT_ERROR
  #define FLOATING_POINT_ERROR 1e-7
#endif

#include <algorithm> // min
#include "../common/macro.h"

namespace math
{
  // Predefinitions
  struct vec3;
  struct mat;
  struct mat4;
  struct quat;

  /* ----- NUMERIC TYPES ----- */
  struct vec3
  {

    vec3();
    vec3(float x, float y, float z);
    float x, y, z;

    bool operator== (const vec3 & v2) const;
    bool operator!= (const vec3 & v2) const;

    void operator= (const vec3 & v2);
    void operator+= (const vec3 & v2);

    vec3 operator- (const vec3 & v2) const;
    vec3 operator+ (const vec3 & v2) const;
    vec3 operator* (const mat4 & m) const;
    vec3 operator* (const float scalar) const;
    vec3 operator/ (const float scalar) const;
    vec3 operator- () const;

    float	length() const;
    float	unsqrt_length() const;
    vec3 normalized() const;

    static float dot(const vec3 & v1, const vec3 & v2);
    static vec3 cross(const vec3 & v1, const vec3 & v2);

    /*
    static vec3 getTranslation(const Matrix4f & m) { return Vector3(m[0][3], m[1][3], m[2][3]); }
    static vec3 getRotationAxis(const quat & q) { return vec3(q.x, q.y, q.z).normalized(); }
    static vec3 getEulerAngles(const quat & q) { return vec3(atan2(2 * (q.w*q.x + q.y*q.z), 1 - (2 * (q.x*q.x + q.y*q.y))), asin(2 * (q.w*q.y - q.z*q.x)), atan2(2 * (q.w*q.z + q.x*q.y), 1 - (2 * (q.y*q.y + q.z*q.z)))); }
    */
  };
  struct mat
  {
    float * data;
    int nbrOfRow;
    int nbrOfCol;

    mat();
    mat(int row, int col, float scalar);
    mat(int row, int col);
    mat(int row, int col, float tab[]);
    mat(const vec3 & v);
    mat(const mat & m);
    ~mat();

    float * operator[] (int row);
    float * operator[] (int row) const;
    mat  operator*  (float scalar) const;
    mat  operator/  (float scalar) const;
    void operator*= (float scalar);
    void operator/= (float scalar);
    mat  operator+  (const mat & m) const;
    mat  operator-  (const mat & m) const;
    void operator+= (const mat & m) const;
    void operator-= (const mat & m);
    mat  operator*  (const mat & m) const;
    void operator*= (const mat & m);
    mat & operator=  (const mat & m);
    bool operator== (const mat & m);
    bool operator!= (const mat & m);

    mat inverse() const;
    mat pinverse() const
    {
      const math::mat & m = *this;
      math::mat mT = math::mat::transpose(m);
      math::mat tmp = m * mT;
      tmp += math::mat(tmp.nbrOfRow, tmp.nbrOfCol, 0.000001f); // <- avoid 0 in the diagonal
      return mT * tmp.inverse();
    }
    static mat inverse(const mat & m);
    static mat pinverse(const mat & m)
    {
      return m.pinverse();
    }

    mat transpose() const;
    static mat transpose(const mat & m);

  private:
    void multiplyRow(int row, float scalar);
    void swapRows(int row1, int row2);
    void addRows(int row1, int row2);
    void multipleRowAddition(int row1, int row2, float nbrOfAdditions);
    void substractRows(int row1, int row2);
    void multipleRowSubstraction(int row1, int row2, float nbrOfSubstractions);
  };
  struct mat4 : public mat
  {
    mat4();
    mat4(mat & m);
    mat4(float tab[]);
    mat4(float scalar);

    vec3 operator*(const vec3 & v);
    void operator=(const mat & m);
    mat  operator+  (const mat & m) const;
    mat  operator-  (const mat & m) const;
    void operator+= (const mat & m) const;
    void operator-= (const mat & m);
    mat  operator*  (const mat & m) const;
    void operator*= (const mat & m);
    bool operator== (const mat & m);
    bool operator!= (const mat & m);

    static mat4 R_Transform(const quat & q);
    static mat4 R_Transform(float angle, const math::vec3 & axis);
    static mat4 T_Transform(const vec3 & t);
    static mat4 S_Transform(const vec3 & s);
  };

  extern vec3 operator*(const mat4 & m, const vec3 & v);
  
  struct quat
  {
    float w, x, y, z;

    quat();
    quat(float f1, float f2, float f3, float f4);
    quat(float t[4]);
    quat(float f, math::vec3 v);

    quat operator-() const;
    quat operator*(float s) const;
    quat operator*(const quat & q2) const;
    quat operator+(const quat & q2) const;
    
    bool operator==(const quat & q2) const;
    bool operator!=(const quat & q2) const;

    /*quat conjugate() const;
    quat reciprocal() const;
    float unsqrt_norm() const;
    float norm() const;
    bool is_unit() const;*/

    static quat R_quat(float radian, const vec3 & axis);
    static quat extractQuat(const mat4 & m);
    static quat slerp(const quat & q1, const quat & q2, float alpha);
  };

  /* -----  ----- */

  /* 3x2 matrix */
  extern mat triangleProjectionMatrix(const math::vec3 & x1, const math::vec3 & x2, const math::vec3 & x3);
}

#endif //__H__MATH__