#pragma once

#ifndef main_h
#define main_h

/* Includes ------------------------------------------------------------------*/
#pragma region basic inlcudes
#include <stdio.h>
#include <stdbool.h> 
#include <string.h>
#include "stm32f4xx_hal.h"
#pragma endregion basic inlcudes

#pragma region freeRTOS includes
#include "cmsis_os.h"
#pragma endregion

#include "rsvISR.h"
#include "userISR.h"
#include "userSysTick.h"
#include "system_clock_rcc.h"
#include "pinConfig.h"
#include "gpio_msp.h"
#include "bsp_led.h"
#include "setup.h"
#include "communication/uart_init.h"
#include "watchdog.h"
#include "adc.h"
#include "tim.h"
#include "communication/usb_computer.h"
#include "communication/can.h"
#include "w25qxx.h"

#pragma region lib
#include "printf.h" // 解决嵌入式系统对动态内存分配的不好支持[造成内存泄漏]...，而RTOS并没有对这些库文件有好的支持，造成hardfault
#pragma endregion

extern osThreadId led_thread_id;
extern osThreadId uart_thread_id;
extern osThreadId setup_thread_id;
extern osThreadId dataAquisition_thread_id;

#endif
