#ifndef __H__MOTION_SYSTEM__
#define __H__MOTION_SYSTEM__

#include "../maths/math.h"

#ifndef PHYSICS_TIME_STEP
  #define PHYSICS_TIME_STEP 0.0005f
  #define INV_PHYSICS_TIME_STEP (1.f / PHYSICS_TIME_STEP)
#endif

#ifndef PHYSICS_DAMPING_FACTOR
  #define PHYSICS_DAMPING_FACTOR .95f
#endif

// namespace physics
//{
  typedef struct
  {
    math::vec3 p; // position
    math::vec3 v; // velocity
    math::vec3 f; // force
    float im; // inverse of mass
  } LINEAR_DATA;

  class LinearMotionSystem
  {
  private:
    size_t linearData_maxNbr;
    size_t linearData_firstUnused;

    LINEAR_DATA  * linearData_pool;
    bool * linearData_used;
    bool * linearData_skip;

  public:
    LinearMotionSystem(size_t size_linear_data_pool);
    ~LinearMotionSystem();

    size_t new_linear_data(const math::vec3 & position, const math::vec3 & velocity, float mass);
    void free_linear_data(size_t i);

    void stop_linear_update(size_t i);
    void resume_linear_update(size_t i);
    void move_linear_position(size_t i, const math::vec3 & offset);
    void add_force(size_t i, const math::vec3 & force);
    void add_velocity(size_t i, const math::vec3 & velocity);
    void apply_impulse(size_t i, const math::vec3 & impulse);
    void set_mass(size_t i, float mass);

    math::vec3 get_linear_position(size_t i);
    math::vec3 get_linear_velocity(size_t i);

    void update_data();
    bool wrong_init();
  };
//}

#endif