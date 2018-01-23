#ifndef __H__CAMERA_CONTROLLER__
#define __H__CAMERA_CONTROLLER__

#include "Camera.h"
#include "./../maths/math.h"

typedef enum
{
  MOVE_UP         = 0,
  MOVE_DOWN       = 1,
  MOVE_LEFT       = 2,
  MOVE_RIGHT      = 3,
  MOVE_FORWARD    = 4,
  MOVE_BACKWARD   = 5,
  ZOOM_IN         = 6,
  ZOOM_OUT        = 7,
  ROTATION_X_CW   = 8,
  ROTATION_Y_CW   = 9,
  ROTATION_X_CCW  = 10,
  ROTATION_Y_CCW  = 11,
  CENTER_VIEW     = 12
} CAMERA_ACTION;
#define COUNT_CAMERA_ACTIONS 16

class CameraController
{
public:
  CameraController(Camera3D & c);

  void init_speeds(float Ts, float Rs, float Zs);
  void init_camera(float fov, float aspect, float near, float far);

  void log_action(CAMERA_ACTION a);
  void unlog_action(CAMERA_ACTION a);
  void clear_logs();

  void update_camera(float dT);

  int actionkey[COUNT_CAMERA_ACTIONS];
private:
  Camera3D & camera;
  
  float T_speed;
  float R_speed;
  float Z_speed;

  bool log[COUNT_CAMERA_ACTIONS];
};

#endif //__H__CAMERA_CONTROLLER__