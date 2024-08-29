#include "rc_control.h"
#include "systim.h"

bool rc_updated;
RC_input_t rc_input;
RC_ctrl_t rc_ctrl;

TimeTag rc_interval = {
    .tag = 0,
    .period = 50,
};

void rc_control_init(void)
{
    rc_updated = false;
}
//将PPM转成rc的输入信号
static void ppm_to_rc()
{
    rc_input.ch1 = ;
}

void pilot_cmd_collect()
{
    rc_updated = false;
    if (check_timetag(&rc_interval)) 
    {
        rc_ctrl.ch1 = rc_input.ch1;
        rc_ctrl.ch2 = rc_input.ch2;
        rc_ctrl.ch3 = rc_input.ch3;
        rc_ctrl.ch4 = rc_input.ch4;
        rc_ctrl.ch5 = rc_input.ch5;
        rc_ctrl.ch6 = rc_input.ch6;
        if(rc_input.sw1 == RC_SW_UP)
        {
           rc_ctrl.status = Arm;
        }
        else
        {
            rc_ctrl.status = Disarm;
        }
        //模式判断
        if(rc_input.sw1 == )
        {
                rc_ctrl.mode = 
        }
        else if(rc_input.sw1 == )
        {

        }
        // rc_ctrl.timestamp = systime_now_ms();
        rc_updated = true;
    }
}