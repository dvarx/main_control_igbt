/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== pwmled2.c ========
 */
/* For usleep() */
#include <unistd.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/PWM.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include "uart_comm.h"
#include <string.h>
#include <stdlib.h>

//global variables
int counter=0;
ADC_Handle   adc_handle;
ADC_Params   adc_params;
int_fast16_t adc_res;
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;
PWM_Handle pwm_handle_duty_a = NULL;
PWM_Params pwm_params_duty;
PWM_Handle pwm_handle_speed_top = NULL;
PWM_Handle pwm_handle_speed_bottom = NULL;
PWM_Params pwm_params_speed;
bool command_to_be_processed;
UART_Handle uart;
UART_Params uartParams;

//uart related parameters
extern uint8_t input_pointer;
extern char input_buffer[];
const char SWITCH_STATE[]="!sws";
const char SET_DUTY_CMD[]="!dut";
const char SET_FREQ_CMD[]="!frq";

//discrete controller state variables
struct pi_controller_32{
    float state_integrator;
    float x;
    float y;
    float kp;
    float ki;
    float delta_T;
};

struct pi_controller_32 current_controller;

enum controller_state{READY,RUNNING,FAULT};
enum controller_fault_type{NO_FAULT,BOTTOM_IGBT_FAULT,TOP_IGBT_FAULT,OTHER_FAULT};
enum controller_state state=READY;
enum controller_state next_state=READY;
enum controller_fault_type fault_type=NO_FAULT;

//function declarations
void timerCallback(Timer_Handle myHandle);
void init_gpio();
void init_timer();
void callback_btn1(uint_least8_t);
void callback_btn2(uint_least8_t);
void gateDriverFaultCallback(uint_least8_t);
inline void update_pi_32(struct pi_controller_32 c,float x_new){
    c.state_integrator=c.state_integrator+c.delta_T*c.y;
    c.x=x_new;
    c.y=c.state_integrator+c.x;
}


void init_gpio(void){
    //debugging
    GPIO_setConfig(CONFIG_GPIO_P32, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG_GPIO_TOP_RDY_1, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG_GPIO_TOP_RDY_2, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG_GPIO_BOT_RDY_1, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG_GPIO_BOT_RDY_2, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG_GPIO_SIGI, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG_GPIO_SIGO, GPIO_CFG_IN_NOPULL);

    GPIO_setConfig(CONFIG2_GPIO_TOP_RDY2, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG2_GPIO_TOP_RDY1, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG2_GPIO_BOT_RDY1, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG2_GPIO_BOT_RDY2, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG2_GPIO_SIGO, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG2_GPIO_SIGI, GPIO_CFG_IN_NOPULL);

    GPIO_setConfig(CONFIG_GPIO_TMPCLK, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG_GPIO_TMPMEAS_A, GPIO_CFG_IN_NOPULL);
    GPIO_setConfig(CONFIG_GPIO_TMPMEAS_B, GPIO_CFG_IN_NOPULL);
    //when using a GPIO pin for output, an extra call to GPIO_setConfig is necessary
    GPIO_setConfig(CONFIG_GPIO_HEARTBEAT, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_RED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_GREEN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_OD_NOPULL);
    GPIO_setConfig(CONFIG_GPIO_LED_BLUE, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_SW1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_SW2, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_TOP_FLT, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);      //need to be pull up to disable red  diode
    GPIO_setConfig(CONFIG_GPIO_BTN_FLT, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);      //needs to be pull up to disable red diode
    GPIO_setConfig(CONFIG2_GPIO_EN, GPIO_CFG_IN_NOPULL);
    GPIO_setCallback(CONFIG_GPIO_TOP_FLT, gateDriverFaultCallback);
    GPIO_setCallback(CONFIG_GPIO_BTN_FLT, gateDriverFaultCallback);
}

void init_timer(void){
    Timer_Handle timer0;
    Timer_Params params;

    /* Call driver init functions */
    Timer_init();

    /* Setting up the timer in continuous callback mode that calls the callback
     * function every 1,000,000 microseconds, or 1 second.
     */
    Timer_Params_init(&params);
    params.period = 200;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }

    return (NULL);
}

void init_adc(void){
    ADC_init();
    ADC_Params_init(&adc_params);
    adc_handle = ADC_open(CONFIG_ADC_ISENSE, &adc_params);
}

void pwm_init(void){
    PWM_init();
    //init pwm for igbt duty cycle
    PWM_Params_init(&pwm_params_duty);
    pwm_params_duty.dutyUnits=PWM_DUTY_FRACTION;
    pwm_params_duty.periodUnits=PWM_PERIOD_HZ;
    pwm_params_duty.dutyValue=0;
    pwm_params_duty.periodValue = 2500;
    pwm_handle_duty_a=PWM_open(CONFIG_PWM_DUTY_A,&pwm_params_duty);
    if(pwm_handle_duty_a==NULL){
        while(1){}
    }
    PWM_start(pwm_handle_duty_a);
    PWM_setDuty(pwm_handle_duty_a, PWM_DUTY_FRACTION_MAX*0.5);
    //init pwm for speed signal
    PWM_Params_init(&pwm_params_speed);
    pwm_params_speed.dutyUnits=PWM_DUTY_FRACTION;
    pwm_params_speed.periodUnits=PWM_PERIOD_HZ;
    pwm_params_speed.dutyValue=0;
    pwm_params_speed.periodValue = 120000;
    pwm_handle_speed_top=PWM_open(CONFIG_PWM_SPEED_TOP,&pwm_params_speed);
    pwm_handle_speed_bottom=PWM_open(CONFIG_PWM_SPEED_BTN,&pwm_params_speed);
    //CONFIG_PWM_SPEED_BTN
    if((pwm_handle_speed_top==NULL)||(pwm_handle_speed_bottom==NULL)){
        while(1){}
    }
    PWM_start(pwm_handle_speed_top);
    PWM_start(pwm_handle_speed_bottom);
    PWM_setDuty(pwm_handle_speed_top, PWM_DUTY_FRACTION_MAX*0.5);
    PWM_setDuty(pwm_handle_speed_bottom, PWM_DUTY_FRACTION_MAX*0.5);
}

/* sets the output color depending on current state */
void setOutputColor(void){
    switch(state){
    case READY:
        GPIO_write(CONFIG_GPIO_LED_RED,0);
        GPIO_write(CONFIG_GPIO_LED_GREEN,0);
        GPIO_toggle(CONFIG_GPIO_LED_BLUE);
        break;
    case RUNNING:
        GPIO_write(CONFIG_GPIO_LED_RED,0);
        GPIO_toggle(CONFIG_GPIO_LED_GREEN);
        GPIO_write(CONFIG_GPIO_LED_BLUE,0);
        break;
    case FAULT:
        GPIO_toggle(CONFIG_GPIO_LED_RED);
        GPIO_write(CONFIG_GPIO_LED_GREEN,0);
        GPIO_write(CONFIG_GPIO_LED_BLUE,0);
        break;
    }
}

void parse_input(uint8_t buffer_size){
    if(!(strncmp(&input_buffer,SWITCH_STATE,sizeof(SWITCH_STATE)/sizeof(char)-1))){
        if(state==READY){
            next_state=RUNNING;
        }
        else if(state==RUNNING){
            next_state=READY;
        }
        return;
    }
    else if(!(strncmp(input_buffer,SET_DUTY_CMD,sizeof(SET_DUTY_CMD)/sizeof(char)-1))){
        uint32_t des_dut=(uint32_t)(atoi(input_buffer+sizeof(SET_DUTY_CMD)/sizeof(char)-1));
        if(des_dut>1024)
            des_dut=1024;
        //change duty cycle
        PWM_setDuty(pwm_handle_duty_a, PWM_DUTY_FRACTION_MAX/1024*des_dut);
        return;
    }
}

/* Callback used for toggling the LED. */
void timerCallback(Timer_Handle myHandle){
    //toggle GPIO
    GPIO_toggle(CONFIG_GPIO_HEARTBEAT);
    //process command
    if(command_to_be_processed){
        parse_input(input_pointer);
        command_to_be_processed=false;
    }
    //set main state machine outputs
    state=next_state;
    switch(state){
    case READY:
        GPIO_write(CONFIG_GPIO_EN,1);
        GPIO_write(CONFIG2_GPIO_EN,1);
        break;
    case RUNNING:
        GPIO_write(CONFIG_GPIO_EN,0);
        GPIO_write(CONFIG2_GPIO_EN,0);
        break;
    case FAULT:
        GPIO_write(CONFIG_GPIO_EN,1);
        GPIO_write(CONFIG2_GPIO_EN,1);
        break;
    }
    //update output color
    setOutputColor();
}

/* Callback for gate driver fault */
void gateDriverFaultCallback(uint_least8_t index){
    if(index==CONFIG_GPIO_TOP_FLT)
        fault_type=TOP_IGBT_FAULT;
    else if(index==CONFIG_GPIO_BTN_FLT)
        fault_type=BOTTOM_IGBT_FAULT;
    else
        fault_type=OTHER_FAULT;
    state=FAULT;
}


/*
 *  ======== mainThread ========
 *  Task periodically increments the PWM duty for the on board LED.
 */
void *mainThread(void *arg0)
{
    init_gpio();
    init_adc();
    init_timer();
    pwm_init();
    init_uart();

    GPIO_enableInt(CONFIG_GPIO_SW1);
    GPIO_enableInt(CONFIG_GPIO_SW2);

    /* output SMCLK at P7.0 for debugging */
    P7->DIR |= BIT0;
    P7->SEL0 |= BIT0;
    P7->SEL1 &= (~BIT0);

    /* Loop forever incrementing the PWM duty */
    while (1) {
        usleep(10000);
    }
}
