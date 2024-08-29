#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H
#include "fms_interface.h"
#include "pid.h"
#include "main.h"
#include "ins_interface.h"
typedef enum
{
  Disarm = 0,           
  Arm,              
} fms_status;

typedef enum
{
  Stabilize = 0,         
  Atthold,
  Depthhold,            
} fms_mode;

typedef struct {
    PID_CLASS roll_pid;
    PID_CLASS pitch_pid;
    PID_CLASS yaw_pid;
    PID_CLASS roll_rate_pid;
    PID_CLASS pitch_rate_pid;
    PID_CLASS yaw_rate_pid;
    float exp_roll;
    float exp_pitch;
    float exp_yaw;
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
} attitude_controller_t;

typedef struct {
    PID_CLASS depth_pid;
    PID_CLASS depth_vel_pid;
    float exp_depth;
    float cal_depth;
    float last_depth;
    float dt;
    float   throttle_thrust;            // throttle thrust input value, +/- 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   lateral_thrust;             // lateral thrust input value, +/- 1.0
} depth_controller_t;

typedef struct
{
  INS_Bus *ctrl_ins_bus;
  FMS_Bus *ctrl_fms_bus;
  fms_mode mode;
  fms_mode last_mode;
  attitude_controller_t att_controller;
  depth_controller_t depth_controller;
} CTRL_Bus;





#endif