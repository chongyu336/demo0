#ifndef _UART_INIT_H
#define _UART_INIT_H

#include "uart_computer.h"
#include "uart_miniSVP.h"
#include "uart_imu.h"
#include "uart_atkTOF.h"

void uart_init(void);
extern CommunicationProtocol *userComm;
extern UartComputer uartComputer;
extern UartIMU uartImu; 
extern UartComputer uart485; // uart6
extern UartAtkTof uartTof;  // uart3

#endif