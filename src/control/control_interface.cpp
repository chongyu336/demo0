#include "control_interface.h"



#define Speed_x_Max 0.5
#define Speed_y_Max 0.5
#define Speed_z_Max 0.5
#define Acc_z_Max 0.1

#define Roll_Max 0.34
#define Pitch_Max 0.34
#define Yaw_Max 0.785
#define Yaw_Rate_Max 2.65


#define MOTORS_MOT_1 0U
#define MOTORS_MOT_2 1U
#define MOTORS_MOT_3 2U
#define MOTORS_MOT_4 3U
#define MOTORS_MOT_5 4U
#define MOTORS_MOT_6 5U

#define MOTORS_MAX_NUM_MOTORS 6

float motor1_factor[MOTORS_MAX_NUM_MOTORS] = {0,              0,              1.0f,           0,                  -1.0f,              1.0f}; 
float motor2_factor[MOTORS_MAX_NUM_MOTORS] = {0,              0,              -1.0f,          0,                  -1.0f,              -1.0f};
float motor3_factor[MOTORS_MAX_NUM_MOTORS] = {0,              0,              -1.0f,          0,                  1.0f,               1.0f};
float motor4_factor[MOTORS_MAX_NUM_MOTORS] = {0,              0,              1.0f,           0,                  1.0f,               -1.0f};
float motor5_factor[MOTORS_MAX_NUM_MOTORS] = {1.0f,           0,              0,              -1.0f,              0,                  0};
float motor6_factor[MOTORS_MAX_NUM_MOTORS] = {-1.0f,          0,              0,              -1.0f,              0,                  0};


// float yawstick, pitchstick, rollstick, throttlestick, forwardstick, laterallstick;
float yawRate, pitchRate, rollRate , zRate;

extern FMS_Bus fms_bus;
extern INS_Bus ins_bus;

// attitude_controller_t att_controller;
// depth_controller_t depth_controller;
CTRL_Bus ctrl_bus;
//controller

float motor_out[MOTORS_MAX_NUM_MOTORS];

#define constrain_float(val, min_val, max_val) (val < min_val ? min_val : (val > max_val ? max_val : val))

static float norm_input(int16_t radio_in)
{
    float ret;
    if (radio_in < JOYSTICK_READ_NEUTRAL) {
        if (JOYSTICK_READ_MIN >= JOYSTICK_READ_NEUTRAL) {
            return 0.0f;
        }
        ret =(radio_in - JOYSTICK_READ_NEUTRAL) / (JOYSTICK_READ_NEUTRAL - JOYSTICK_READ_MIN);
    } else {
        if (JOYSTICK_READ_MAX <= JOYSTICK_READ_NEUTRAL) {
            return 0.0f;
        }
        ret = (radio_in - JOYSTICK_READ_NEUTRAL) / (JOYSTICK_READ_MAX  - JOYSTICK_READ_NEUTRAL);
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

static float Limit(float pwm, float min, float max)
{
    return pwm < min ? min : (pwm > max ? max : pwm);
}

static float absself(float num)
{
    if(num > 0)
    {
        return num;
    }
    else{
        return -num;
    }
}
static float deadzone_range(float num1, float num2)
{
    if( absself(num1 - JOYSTICK_READ_NEUTRAL ) < num2)
    {
        return 0;
    }
    else{
        return num1;
    }
}

void control_interface_init(void)
{
    // 初始化姿态控制器PID
    ctrl_bus.att_controller.roll_pid.init();
    ctrl_bus.att_controller.roll_pid.isIncrement = true;
    ctrl_bus.att_controller.pitch_pid.init();
    ctrl_bus.att_controller.pitch_pid.isIncrement = true;
    ctrl_bus.att_controller.yaw_pid.init();
    ctrl_bus.att_controller.yaw_pid.isIncrement = true;

    ctrl_bus.att_controller.roll_rate_pid.init();
    ctrl_bus.att_controller.roll_rate_pid.isIncrement = true;
    ctrl_bus.att_controller.pitch_rate_pid.init();
    ctrl_bus.att_controller.pitch_rate_pid.isIncrement = true;
    ctrl_bus.att_controller.yaw_rate_pid.init();
    ctrl_bus.att_controller.yaw_rate_pid.isIncrement = true;
    ctrl_bus.att_controller.exp_roll = ctrl_bus.att_controller.exp_pitch = ctrl_bus.ctrl_bus.exp_yaw = 0.0f;

    // 初始化深度控制器PID
    ctrl_bus.depth_controller.depth_pid.init();
    ctrl_bus.depth_controller.depth_pid.isIncrement = true;
    ctrl_bus.depth_controller.depth_vel_pid.init();
    ctrl_bus.depth_controller.depth_vel_pid.isIncrement = true;
    ctrl_bus.depth_controller.exp_depth = 0.0f;
    ctrl_bus.depth_controller.cal_depth = 0.0f;
    ctrl_bus.depth_controller.last_depth = 0.0f;
    ctrl_bus.depth_controller.dt = 0.0f;

    //初始化输出
    ctrl_bus.depth_controller.forward_thrust = ctrl_bus.depth_controller.lateral_thrust = ctrl_bus.depth_controller.throttle_thrust = ctrl_bus.att_controller.roll_thrust = ctrl_bus.att_controller.pitch_thrust = ctrl_bus.att_controller.yaw_thrust = 0.0f;
}



static void control_set_mode(controller_t * set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }

    set_mode->status = fms_output_ctrl.output_status;
    set_mode->last_mode = set_mode->mode;
    set_mode->mode = fms_output_ctrl.output_mode;
}


static void mode_change_control_transit(controller_t * mode_change)
{
    if (mode_change == NULL)
    {
        return;
    }
        //杆位输入，PWM值 1000-2000
    float rollstick = deadzone_range(mode_change->RC_ctrl->ch1, 50);
    float pitchstick = deadzone_range(mode_change->RC_ctrl->ch2,50);
    float throttlestick = deadzone_range(mode_change->RC_ctrl->ch3,50);
    float yawstick = deadzone_range(mode_change->RC_ctrl->ch4,50);
    float forwardstick = deadzone_range(mode_change->RC_ctrl->ch5,50);
    float laterallstick = deadzone_range(mode_change->RC_ctrl->ch6,50);    

        //杆位输入，-1.0-1.0
    float roll_desired = norm_input(rollstick)*Roll_Max;
    float pitch_desired = norm_input(pitchstick)*Pitch_Max;
    float yaw_desired = norm_input(yawstick)*Yaw_Rate_Max;
    float forwardout = norm_input(forwardstick) *Speed_x_Max;
    float laterallout = norm_input(laterallstick)*Speed_y_Max;

    float depDT = mode_change->INS_depth_point[1];
    mode_change->depth_controller.dt = depDT;
    mode_change->depth_controller.cal_depth += norm_input(throttlestick)*Speed_z_Max * depDT;
    float depth_desired = mode_change->depth_controller.cal_depth;

    mode_change->depth_controller.forward_thrust = forwardout;
    mode_change->depth_controller.laterall_thrust = laterallout;

    //传感器角度数据
    float roll_feedback = mode_change->INS_angle_point[0];
    float pitch_feedback = mode_change->INS_angle_point[1];
    float yaw_feedback = mode_change->INS_angle_point[2];

    // //传感器角速度数据
    // float roll_rate_feedback = mode_change->INS_gyro_point[0];
    // float pitch_rate_feedback = mode_change->INS_gyro_point[1];
    // float yaw_rate_feedback = mode_change->INS_gyro_point[2];
    // if(yaw_feedback > 180)
    // {
    //     yaw_feedback = yaw_feedback - 360;
    //     yaw_rate_feedback = - yaw_rate_feedback;
    // }

    //深度反馈
    float depth_feedback = mode_change->INS_depth_point[0];

    if ((mode_change->last_mode == Stabilize || mode_change->last_mode == Depthhold) && mode_change->mode == Atthold)
    {
        mode_change->att_controller.exp_roll = roll_feedback;
        mode_change->att_controller.exp_pitch = pitch_feedback;
        mode_change->att_controller.exp_yaw = yaw_feedback;
        mode_change->depth_controller.exp_depth = depth_desired;
    }
    else if ((mode_change->last_mode == Stabilize || mode_change->last_mode == Atthold) && mode_change->mode == Depthhold)
    {
        mode_change->att_controller.exp_roll = roll_desired;
        mode_change->att_controller.exp_pitch = pitch_desired;
        mode_change->att_controller.exp_yaw = yaw_desired;
        mode_change->depth_controller.exp_depth = depth_feedback;
    }
    else if(mode_change->last_mode == Atthold && mode_change->mode == Atthold)
    {
        mode_change->depth_controller.exp_depth = depth_desired;
    }
    else if(mode_change->last_mode == Depthhold && mode_change->mode == Depthhold)
    {
        mode_change->att_controller.exp_roll = roll_desired;
        mode_change->att_controller.exp_pitch = pitch_desired;
        mode_change->att_controller.exp_yaw = yaw_desired;
    }
    else{
        mode_change->att_controller.exp_roll = roll_desired;
        mode_change->att_controller.exp_pitch = pitch_desired;
        mode_change->att_controller.exp_yaw = yaw_desired;
        mode_change->depth_controller.exp_depth = depth_desired;
    }
}
// static void att_control(attitude_controller_t * attctrl, float roll, float pitch, float yaw)
// {
//     float rollfeedback = attctrl->roll_pid.fb;
//     float pitchfeedback = attctrl->pitch_pid.fb;
//     float yawfeedback = attctrl->yaw_pid.fb;
//     float rollratefeedback = attctrl->roll_rate_pid.fb;
//     float pitchratefeedback = attctrl->pitch_rate_pid.fb;
//     float yawratefeedback = attctrl->yaw_rate_pid.fb;

//     attctrl->roll_pid.ref = roll;
//     attctrl->pitch_pid.ref = pitch;
//     attctrl->yaw_pid.ref = yaw;

//     attctrl->roll_rate_pid.ref = attctrl->roll_pid.calc(rollfeedback);
//     attctrl->pitch_rate_pid.ref =attctrl->pitch_pid.calc(pitchfeedback);
//     attctrl->raw_rate_pid.ref =attctrl->yaw_pid.calc(yawfeedback);

//     attctrl->roll_thrust = attctrl->roll_rate_pid.calc(rollratefeedback);
//     attctrl->pitch_thrust = attctrl->pitch_rate_pid.calc(pitchratefeedback);
//     attctrl->yaw_thrust = attctrl->yaw_rate_pid.calc(yawratefeedback);

// }
// static void depth_control(depth_controller_t * depthctrl, float depth)
// {
    
//     float depthfeedback = depthctrl.depth_pid.fb;
//     float velfeedback = depthctrl.depth_vel_pid.fb;

//     depthctrl->depth_pid.ref = depth;
//     depthctrl->depth_vel_pid.ref = depthctrl->depth_pid.calc(depthfeedback);
//     depthctrl->throttle_thrust = depthctrl->depth_vel_pid.calc(velfeedback);

// }
static void update_data(controller_t * update_data)
{
    if (update_data == NULL)
    {
        return;
    }
    update_data->att_controller.roll_pid.fb = update_data->INS_angle_point[0];
    update_data->att_controller.pitch_pid.fb = update_data->INS_angle_point[1];
    update_data->att_controller.yaw_pid.fb = update_data->INS_angle_point[2];
    update_data->att_controller.roll_rate_pid.fb = update_data->INS_gyro_point[0];
    update_data->att_controller.pitch_rate_pid.fb = update_data->INS_gyro_point[1];
    update_data->att_controller.yaw_rate_pid.fb = update_data->INS_gyro_point[2];
    update_data->depth_controller.depth_pid.fb = update_data->INS_depth_point[0];

    float depth_feedback = update_data->INS_depth_point[0];
    float depDT = update_data->INS_depth_point[1];
    float ddepth = depth_feedback - update_data->depth_controller.last_depth;
    float depth_rate_feedback = ddepth / depDT;
    update_data->depth_controller.last_depth = depth_feedback;
    update_data->depth_controller.depth_vel_pid.fb = depth_rate_feedback;

    update_data->att_controller.roll_pid.ref = update_data->att_controller.exp_roll;
    update_data->att_controller.pitch_pid.ref = update_data->att_controller.exp_pitch;
    update_data->att_controller.yaw_pid.ref = update_data->att_controller.exp_yaw;
    update_data->depth_controller.depth_pid.ref = update_data->depth_controller.exp_depth;


}

static void cal_thrust(controller_t * cal_thrust)
{
    if (cal_thrust == NULL)
    {
        return;
    }
    cal_thrust->att_controller.roll_rate_pid.ref = cal_thrust->att_controller.roll_pid.calc(cal_thrust->att_controller.roll_pid.fb);
    cal_thrust->att_controller.pitch_rate_pid.ref = cal_thrust->att_controller.pitch_pid.calc(cal_thrust->att_controller.pitch_pid.fb);
    cal_thrust->att_controller.yaw_rate_pid.ref = cal_thrust->att_controller.yaw_pid.calc(cal_thrust->att_controller.yaw_pid.fb);

    cal_thrust->att_controller.roll_thrust = cal_thrust->att_controller.roll_rate_pid.calc(cal_thrust->att_controller.roll_rate_pid.fb);
    cal_thrust->att_controller.pitch_thrust = cal_thrust->att_controller.pitch_rate_pid.calc(cal_thrust->att_controller.pitch_rate_pid.fb);
    cal_thrust->att_controller.yaw_thrust = cal_thrust->att_controller.yaw_rate_pid.calc(cal_thrust->att_controller.yaw_rate_pid.fb);

    cal_thrust->depth_controller.depth_vel_pid.ref = cal_thrust->depth_controller.depth_pid.calc(cal_thrust->depth_controller.depth_pid.fb);
    cal_thrust->depth_controller.throttle_thrust = cal_thrust->depth_controller.depth_vel_pid.calc(cal_thrust->depth_controller.depth_vel_pid.fb);
}

void fms_interface_step(uint32_t timestamp)
{   
    ctrl_bus.ctrl_ins_bus = &ins_bus;
    ctrl_bus.ctrl_fms_bus = &fms_bus;
    ctrl_bus.last_mode = ctrl_bus.mode;
    ctrl_bus.mode = ctrl_bus.ctrl_fms_bus->fms_cmd_bus->mode;
    control_set_mode(&controller);
    if(controller.status == Arm)
    {
        mode_change_control_transit(&controller);

        update_data(&controller);

        cal_thrust(&controller);

        thrust_alloc(&controller);
        motor_out[MOTORS_MOT_1] = controller.att_controller.roll_thrust * motor1_factor[0] + controller.att_controller.pitch_thrust * motor1_factor[1] + controller.att_controller.yaw_thrust * motor1_factor[2] + controller.depth_controller.throttle_thrust * motor1_factor[3]+ controller.depth_controller.forward_thrust * motor1_factor[4] + controller.depth_controller.laterall_thrust * motor1_factor[5] ;

        motor_out[MOTORS_MOT_2] = controller.att_controller.roll_thrust * motor2_factor[0] + controller.att_controller.pitch_thrust * motor2_factor[1] + controller.att_controller.yaw_thrust * motor2_factor[2] + controller.depth_controller.throttle_thrust * motor2_factor[3]+ controller.depth_controller.forward_thrust * motor2_factor[4] + controller.depth_controller.laterall_thrust * motor2_factor[5] ;

        motor_out[MOTORS_MOT_3] = controller.att_controller.roll_thrust * motor3_factor[0] + controller.att_controller.pitch_thrust * motor3_factor[1] + controller.att_controller.yaw_thrust * motor3_factor[2] + controller.depth_controller.throttle_thrust * motor3_factor[3]+ controller.depth_controller.forward_thrust * motor3_factor[4] + controller.depth_controller.laterall_thrust * motor3_factor[5] ;

        motor_out[MOTORS_MOT_4] = controller.att_controller.roll_thrust * motor4_factor[0] + controller.att_controller.pitch_thrust * motor4_factor[1] + controller.att_controller.yaw_thrust * motor4_factor[2] + controller.depth_controller.throttle_thrust * motor4_factor[3]+ controller.depth_controller.forward_thrust * motor4_factor[4] + controller.depth_controller.laterall_thrust * motor4_factor[5] ;

        motor_out[MOTORS_MOT_5] = controller.att_controller.roll_thrust * motor5_factor[0] + controller.att_controller.pitch_thrust * motor5_factor[1] + controller.att_controller.yaw_thrust * motor5_factor[2] + controller.depth_controller.throttle_thrust * motor5_factor[3]+ controller.depth_controller.forward_thrust * motor5_factor[4] + controller.depth_controller.laterall_thrust * motor5_factor[5] ;

        motor_out[MOTORS_MOT_6] = controller.att_controller.roll_thrust * motor6_factor[0] + controller.att_controller.pitch_thrust * motor6_factor[1] + controller.att_controller.yaw_thrust * motor6_factor[2] + controller.depth_controller.throttle_thrust * motor6_factor[3]+ controller.depth_controller.forward_thrust * motor6_factor[4] + controller.depth_controller.laterall_thrust * motor6_factor[5] ;
    }
    else
    {
        for (int i = 0; i < MOTORS_MAX_NUM_MOTORS; i++) {
            motor_out[i] = 0.0f;
        }

    }
    //记录模式
    
    //舵机控制
    
    //***电机分配 */
    set_esc();

        

}

