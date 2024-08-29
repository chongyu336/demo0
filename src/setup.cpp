/************************************************************
 * FileName:        Enable Management.cpp
 * Description:     所有开关的使能控制
 *                  STM32
 * Auther:          Jinsheng
 * CreateDate:      2021-07-12
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include "setup.h"
#include "main.h"
#include "signalAcquisition.h"
// ------------------------------------------------------------------------------------

sAdcVal adcVal;
// 如果需要adc中断处理则使用该函数
static void adc_callback(sAdcVal *val)
{
    adcVal = *val;
}

void vThreadSetup(void *pvParameters)
{
    // uint8_t state = 0;
    char info[256];
    UNUSED(info);

    delay(10); // wait for uart ready
    usb_init();

    adc_received_callback = adc_callback;
    ADC_init();

    can_init();

    uart_init();

    tim_init();

    W25QXX_Init();
    // W25QXX_test_write(info);

    // SignalAquisition::init((CommunicationProtocol **)&userComm);
    //  osThreadDef(dataAquisition, vThreadDataAquisition, osPriority::osPriorityHigh, 0, 512);
    //  dataAquisition_thread_id = osThreadCreate(osThread(dataAquisition), NULL);
    // delay(10);

    // sprintf(info, "starting acquisition system...\n");
    // userComm->send(info);
    // vTaskDelay(10);

    // pwm_shutdown();

    // sampler.powerEnSet(10, 1); // 52V 电源常开

    osDelay(10);

    while (1)
    {
        // adc_softTrigger();

        // can_test(0); // id可以选0或者1

        // float duty = PWM_duty_get(3);
        // sprintf(info, "duty = %0.1f\r\n", duty);
        // usbComm.send(info);

        // sprintf(info, "tof longest distance = %0.3fm\r\n", uartTof.longestDistance);
        // usbComm.send(info);

        // W25QXX_test_read(info);
        // usbComm.send(info);

        osDelay(1000);

        // ADC_DMA_callback();
    }

    // 查看任务stack是否溢出
    // sprintf(info, "heap remainsize %d, minimum freesize %d, stack remainsize %d\n",
    //        (int)xPortGetFreeHeapSize(), (int)xPortGetMinimumEverFreeHeapSize(), (int)uxTaskGetStackHighWaterMark(NULL));
    // sprintf(info, "led stack remainsize %d, uart stack remainsize %d, stack remainsize %d\n",
    //        (int)uxTaskGetStackHighWaterMark(led_thread_id), (int)uxTaskGetStackHighWaterMark(uart_thread_id), (int)uxTaskGetStackHighWaterMark(NULL));
}
