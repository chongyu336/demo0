#ifndef RC_CONTROL_H
#define RC_CONTROL_H

#define JOYSTICK_READ_MIN       1000
#define JOYSTICK_READ_MAX       2000
#define JOYSTICK_READ_NEUTRAL   1500
#define JOYSTICK_OUTPUT_MIN     -1.0f
#define JOYSTICK_OUTPUT_MAX     1.0f


typedef enum
{
    Disarm = 0,
    Arm,
} rc_status;

typedef enum
{       
  Stabilize = 0,  
  Atthold,
  Depthhold,            
} rc_mode;

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
typedef struct
{
    int16_t ch1;   //右侧左右 roll
    int16_t ch2;   //右侧上下 pitch  
    int16_t ch3;   //左侧上下 油门
    int16_t ch4;   //左侧左右 yaw
   
    int16_t ch5;   //前后移动
    int16_t ch6;   //左右移动

    uint8_t sw1;   //SWA，二档
    uint8_t sw2;   //SWB，二档
    uint8_t sw3;   //SWC，三档
    uint8_t sw4;   //SWD，二档
} RC_input_t;

typedef struct
{
    int16_t ch1;   //右侧左右 roll
    int16_t ch2;   //右侧上下 pitch  
    int16_t ch3;   //左侧上下 油门
    int16_t ch4;   //左侧左右 yaw
   
    int16_t ch5;   //前后移动
    int16_t ch6;   //左右移动

    uint32_t timestamp;
    rc_mode mode;   //模式
    rc_status status;   //状态
} RC_ctrl_t;

void rc_control_init(void);

void pilot_cmd_collect();


#endif