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

/* Driver configuration */
#include "ti_drivers_config.h"

//global variables
int counter=0;
ADC_Handle   adc_handle;
ADC_Params   adc_params;
int_fast16_t adc_res;
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;
PWM_Handle pwm_handle_duty_a = NULL;
PWM_Handle pwm_handle_duty_b = NULL;
PWM_Params pwm_params_duty;
PWM_Handle pwm_handle_speed = NULL;
PWM_Params pwm_params_speed;


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

enum controller_state{INIT,READY,RUNNING,FAULT};
enum controller_fault_type{NO_FAULT,BOTTOM_IGBT_FAULT,TOP_IGBT_FAULT,OTHER_FAULT};
enum controller_state state=INIT;
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
    //when using a GPIO pin for output, an extra call to GPIO_setConfig is necessary
    GPIO_setConfig(CONFIG_GPIO_HEARTBEAT, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_RED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_GREEN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_BLUE, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_SW1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_SW2, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_TOP_FLT, GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BTN_FLT, GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setCallback(CONFIG_GPIO_SW1, callback_btn1);
    GPIO_setCallback(CONFIG_GPIO_SW2, callback_btn2);
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
    pwm_handle_duty_b=PWM_open(CONFIG_PWM_DUTY_B,&pwm_params_duty);
    if(pwm_handle_duty_a==NULL){
        while(1){}
    }
    PWM_start(pwm_handle_duty_a);
    PWM_setDuty(pwm_handle_duty_a, 0);
    //init pwm for speed signal
    PWM_Params_init(&pwm_params_speed);
    pwm_params_speed.dutyUnits=PWM_DUTY_FRACTION;
    pwm_params_speed.periodUnits=PWM_PERIOD_HZ;
    pwm_params_speed.dutyValue=0;
    pwm_params_speed.periodValue = 120000;
    pwm_handle_speed=PWM_open(CONFIG_PWM_SPEED_TOP,&pwm_params_speed);
    if(pwm_handle_speed==NULL){
        while(1){}
    }
    PWM_start(pwm_handle_speed);
    PWM_setDuty(pwm_handle_speed, PWM_DUTY_FRACTION_MAX*0.8);
}

//advance state machine
void callback_btn1(uint_least8_t index){
    switch(state){
    case INIT:      state=READY; break;
	/*Hint: Errara in PCB
	- TOP_RDY2 is always low and is therefore not checked
	- TOP_RDY1 <-> BOT_RDY2
	- BOT_RDY1 <-> BOT_RDY2
	- BOT_RDY2 <-> BOT_RDY1
	*/

//    if( GPIO_read(CONFIG_GPIO_TOP_RDY_1)&
//                            1 &
//                            GPIO_read(CONFIG_GPIO_BOT_RDY_1)&
//                            GPIO_read(CONFIG_GPIO_BOT_RDY_1))
    case READY:     state=RUNNING; break;
    case RUNNING:   state=RUNNING; break;
    case FAULT:     state=INIT; break;
    default:        state=INIT; break;
    }
}

//disadvance state machine
void callback_btn2(uint_least8_t index){
    switch(state){
    case INIT:      state=INIT; break;
    case READY:     state=INIT; break;
    case RUNNING:   state=READY; break;
    default:        state=INIT; break;
    }
}

/* sets the output color depending on current state */
void setOutputColor(void){
    switch(state){
    case INIT:
        GPIO_toggle(CONFIG_GPIO_LED_RED);
        GPIO_toggle(CONFIG_GPIO_LED_GREEN);
        GPIO_toggle(CONFIG_GPIO_LED_BLUE);
        break;
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

/* Callback used for toggling the LED. */
void timerCallback(Timer_Handle myHandle){
    //toggle GPIO
    GPIO_toggle(CONFIG_GPIO_HEARTBEAT);
    //measure current
    adc_res = ADC_convert(adc_handle, &adcValue0);
    adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc_handle, adcValue0);
    if(adc_res!=ADC_STATUS_SUCCESS){
        while(1){}
    }

    //set main state machine outputs
    switch(state){
    case INIT:
        GPIO_write(CONFIG_GPIO_EN,0);
        PWM_setDuty(pwm_handle_duty_a, 0);
        PWM_setDuty(pwm_handle_duty_b, 0);
        break;
    case READY:
        GPIO_write(CONFIG_GPIO_EN,0);
        PWM_setDuty(pwm_handle_duty_a, 0);
        PWM_setDuty(pwm_handle_duty_b, 0);
        break;
    case RUNNING:
        GPIO_write(CONFIG_GPIO_EN,1);
        PWM_setDuty(pwm_handle_duty_a, PWM_DUTY_FRACTION_MAX/2);
        PWM_setDuty(pwm_handle_duty_b, PWM_DUTY_FRACTION_MAX/2);
        break;
    case FAULT:
        GPIO_write(CONFIG_GPIO_EN,0);
        PWM_setDuty(pwm_handle_duty_a, 0);
        PWM_setDuty(pwm_handle_duty_b, 0);
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

    GPIO_enableInt(CONFIG_GPIO_SW1);
    GPIO_enableInt(CONFIG_GPIO_SW2);

    /* Loop forever incrementing the PWM duty */
    while (1) {
        usleep(10000);
    }
}
