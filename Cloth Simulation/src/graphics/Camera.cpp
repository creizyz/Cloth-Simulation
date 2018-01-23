#include "Camera.h"

#include "../graphics/openGL.h"
#include "../common/macro.h"

#include <math.h>
#include <algorithm> // std::min, std::max

Camera3D::Camera3D()
{ }
void Camera3D::set_nearAndFarPlane(float near, float far)
{
  DBG_ASSERT(near > 0.f);
  DBG_ASSERT(far > near);
  nearPlane = std::min(0.1f, near);
  farPlane = std::max(nearPlane + 0.1f, far);
}
void Camera3D::set_fieldOfView(float field_of_view)
{
  fov = std::min(2.2f, std::max(0.03f, field_of_view));
}
void Camera3D::increase_fieldOfView(float increment)
{
  set_fieldOfView(fov + increment);
}
void Camera3D::set_position(const math::vec3 & position)
{
  pos = position;
}
void Camera3D::set_viewportAspect(float aspect)
{
  DBG_ASSERT(aspect != 0.f);
  if (aspect != 0.f) viewportAspect = aspect;
}

void Camera3D::translate(const math::vec3 & t)
{
  DBG_VALID_VEC(t);
  pos += (t * math::mat4::transpose(view()));
}
void Camera3D::rotate(float aroundX, float aroundY)
{
  orientX += aroundX;
  orientY += aroundY;
  normalize_angles();
}

void Camera3D::look_at(const math::vec3 & point)
{
  DBG_ASSERT(point != pos);
  if (point != pos)
  {
    math::vec3 dir = (point - pos).normalized();
    orientX = asin(-dir.y);
    orientY = atan2f(-dir.x, -dir.z);
    normalize_angles();
  }
}

math::mat4 Camera3D::matrix()
{
  return projection() * view();
}
math::mat4 Camera3D::view()
{
  return orientation() * math::mat4::T_Transform(-pos);
}
math::mat4 Camera3D::projection()
{
  DBG_VALID_FLOAT(fov);
  DBG_VALID_FLOAT(viewportAspect);
  DBG_VALID_FLOAT(nearPlane);
  DBG_VALID_FLOAT(farPlane);

  math::mat4 projection_matrix(1);
  if (fov > 0 && viewportAspect > 0 && nearPlane != farPlane)
  {
    float tanHalfFov = tan(0.5f * fov);

    projection_matrix.data[0] = 1.f / (tanHalfFov * viewportAspect);
    projection_matrix.data[5] = 1.f / tanHalfFov;
    projection_matrix.data[10] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    projection_matrix.data[11] = -(2.f * farPlane * nearPlane) / (farPlane - nearPlane);
    projection_matrix.data[14] = -1.f;
    projection_matrix.data[15] = 0.f;
  }
  DBG_VALID_MAT4(projection_matrix);
  return projection_matrix;
}
math::mat4 Camera3D::orientation()
{
  return math::mat4::R_Transform(orientX, math::vec3(1.f, 0.f, 0.f)) * math::mat4::R_Transform(orientY, math::vec3(0.f, 1.f, 0.f));
}
  void Camera3D::normalize_angles()
  {
    orientY = fmod(orientY, 360.f);
    orientX = std::min(MAX_VERTICAL_ANGLE, std::max(-MAX_VERTICAL_ANGLE, orientX));
  }