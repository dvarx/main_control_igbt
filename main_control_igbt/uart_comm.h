/*
 * uart_comm.h
 *
 *  Created on: Mar 4, 2021
 *      Author: dvarx
 */

#ifndef UART_COMM_H_
#define UART_COMM_H_

#define __MSP432P401R__

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

void init_uart(void);

void uart_interrupt_handler(void);

#endif /* UART_COMM_H_ */
