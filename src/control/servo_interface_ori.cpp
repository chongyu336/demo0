#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define PWM_DETAL_VALUE 10

#define SERVO_MINUS_PWM_KEY 
#define SERVO_ADD_PWM_KEY 

const RC_ctrl_t *servo_rc;

uint16_t servo_pwm = SERVO_MIN_PWM;


void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();

    while(1)
    {

        if( servo_rc->key.v & SERVO_MINUS_PWM_KEY)
        {
            servo_pwm -= PWM_DETAL_VALUE;
        }
        else if(servo_rc->key.v & SERVO_ADD_PWM_KEY)
        {
            servo_pwm += PWM_DETAL_VALUE;
        }

        //limit the pwm
        //限制pwm
        if(servo_pwm < SERVO_MIN_PWM)
        {
            servo_pwm = SERVO_MIN_PWM;
        }
        else if(servo_pwm > SERVO_MAX_PWM)
        {
            servo_pwm = SERVO_MAX_PWM;
        }

        servo_pwm_set();
        
        osDelay(10);
    }
}


