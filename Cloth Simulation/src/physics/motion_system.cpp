#include "motion_system.h"
#include "../common/macro.h"

// using namespace physics;

LinearMotionSystem::LinearMotionSystem(size_t size_linear_data_pool) :
  linearData_maxNbr(0),
  linearData_pool(nullptr),
  linearData_used(nullptr),
  linearData_skip(nullptr)
{
  try
  {
    linearData_maxNbr = size_linear_data_pool;
    linearData_firstUnused = 0;

    linearData_pool = new LINEAR_DATA[linearData_maxNbr];
    linearData_used = new bool[linearData_maxNbr];
    linearData_skip = new bool[linearData_maxNbr];

    memset(linearData_used, 0, linearData_maxNbr * sizeof(bool));
    memset(linearData_skip, 0, linearData_maxNbr * sizeof(bool));
  }
  catch (const std::exception &)
  {
    SAFE_DELETE_TAB(linearData_pool);
    SAFE_DELETE_TAB(linearData_used);
    SAFE_DELETE_TAB(linearData_skip);

    linearData_maxNbr = 0;
    linearData_pool = nullptr;
    linearData_used = nullptr;
    linearData_skip = nullptr;
  }
}
LinearMotionSystem::~LinearMotionSystem()
{
  SAFE_DELETE_TAB(linearData_pool);
  SAFE_DELETE_TAB(linearData_used);
  SAFE_DELETE_TAB(linearData_skip);
}

size_t LinearMotionSystem::new_linear_data(const math::vec3 & position, const math::vec3 & velocity, float mass)
{
  DBG_VALID_VEC(position);
  DBG_VALID_VEC(velocity);
  DBG_VALID_FLOAT(mass);
  DBG_ASSERT(mass != 0);

  if (mass != 0)
  {
    for (size_t n = linearData_firstUnused; n < linearData_maxNbr; n++)
    {
      if (!linearData_used[n])
      {
        linearData_firstUnused = n + 1;
        linearData_used[n] = true;
        linearData_pool[n].p = position;
        linearData_pool[n].v = velocity;
        linearData_pool[n].im = 1 / mass;
        return n;
      }
    }
  }

  return -1;
}
void LinearMotionSystem::free_linear_data(size_t i)
{
  DBG_ASSERT(i < linearData_maxNbr);
  DBG_ASSERT(linearData_used[i]);

  if (i < linearData_maxNbr)
  {
    linearData_used[i] = false;
    linearData_skip[i] = false;
    if (i < linearData_firstUnused) linearData_firstUnused = i;
  }
}
void LinearMotionSystem::stop_linear_update(size_t i)
{
  DBG_ASSERT(i < linearData_maxNbr);

  if (i < linearData_maxNbr && linearData_used[i])
  {
    linearData_skip[i] = true;
  }
}
void LinearMotionSystem::resume_linear_update(size_t i)
{
  DBG_ASSERT(i < linearData_maxNbr);

  if (i < linearData_maxNbr)
  {
    linearData_skip[i] = false;
  }
}
math::vec3 LinearMotionSystem::get_linear_position(size_t i)
{
  DBG_ASSERT(i < linearData_maxNbr);

  if (i < linearData_maxNbr  && linearData_used[i])
  {
    return linearData_pool[i].p;
  }
  return math::vec3();
}
math::vec3 LinearMotionSystem::get_linear_velocity(size_t i)
{
  DBG_ASSERT(i < linearData_maxNbr);

  if (i < linearData_maxNbr  && linearData_used[i])
  {
    return linearData_pool[i].v;
  }
  return math::vec3();
}
void LinearMotionSystem::move_linear_position(size_t i, const math::vec3 & offset)
{
  DBG_VALID_VEC(offset);

  DBG_ASSERT(i < linearData_maxNbr);
  DBG_ASSERT(linearData_used[i]);

  if (i < linearData_maxNbr)
  {
    linearData_pool[i].p += offset;
    linearData_pool[i].v += offset / PHYSICS_TIME_STEP;
  }
}
void LinearMotionSystem::add_force(size_t i, const math::vec3 & force)
{
  DBG_VALID_VEC(force);
  DBG_ASSERT(i < linearData_maxNbr);

  if (i < linearData_maxNbr)
  {
    linearData_pool[i].f += force;
  }
}
void LinearMotionSystem::add_velocity(size_t i, const math::vec3 & velocity)
{
  DBG_VALID_VEC(velocity);
  DBG_ASSERT(i < linearData_maxNbr);

  if (i < linearData_maxNbr)
  {
    linearData_pool[i].v += velocity;
  }
}
void LinearMotionSystem::apply_impulse(size_t i, const math::vec3 & impulse)
{
  DBG_VALID_VEC(impulse);
  DBG_ASSERT(i < linearData_maxNbr);

  if (i < linearData_maxNbr)
  {
    linearData_pool[i].v += impulse * linearData_pool[i].im;
  }
}
void LinearMotionSystem::set_mass(size_t i, float mass)
{
  DBG_VALID_FLOAT(mass);
  DBG_ASSERT(i < linearData_maxNbr);
  DBG_ASSERT(mass != 0.f);

  if (i < linearData_maxNbr && mass != 0.f)
  {
    linearData_pool[i].im = 1.f / mass;
  }
}

void LinearMotionSystem::update_data()
{
  for (size_t n = 0; n < linearData_maxNbr; n++)
  {
    if (linearData_used[n] && !linearData_skip[n])
    {
      linearData_pool[n].v = linearData_pool[n].v * PHYSICS_DAMPING_FACTOR + linearData_pool[n].f * linearData_pool[n].im * PHYSICS_TIME_STEP;
      linearData_pool[n].p += linearData_pool[n].v * PHYSICS_TIME_STEP;
      linearData_pool[n].f = math::vec3();
    }
  }
}
bool LinearMotionSystem::wrong_init()
{
  return linearData_maxNbr == 0;
}