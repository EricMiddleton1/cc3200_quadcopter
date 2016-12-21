/*
 * Receiver.c
 *
 *  Created on: Dec 19, 2016
 *      Author: eric
 */

#include <stdint.h>

#include "hw_types.h"
#include "hw_memmap.h"
#include "timer.h"
#include "Timer.h"

#define MIN(a, b)           (((a) < (b)) ? (a) : (b))
#define MAX(a, b)           (((a) > (b)) ? (a) : (b))
#define CLAMP(n, a, b)      (MIN((b), MAX((a), (n))))

#define CPU_FREQ            (80000000L)

#define RC_PULSE_MIN        (1 * CPU_FREQ / 1000) //1ms
#define RC_PULSE_MAX        (2 * CPU_FREQ / 1000) //2ms

#define MOTOR_UPDATE_RATE   (50)
#define MOTOR_PWM_TOP       (CPU_FREQ / MOTOR_UPDATE_RATE)
#define MOTOR_MIN_PULSE     (1 * CPU_FREQ / 1000) //1ms
#define MOTOR_MAX_PULSE     (2 * CPU_FREQ / 1000) //2ms

#define RC_TIMEOUT_VALUE    (MOTOR_UPDATE_RATE / 2) //0.5 seconds

volatile static uint32_t _throttleEdge, _yawEdge, _pitchEdge, _rollEdge;
volatile static uint32_t _throttle, _yaw, _pitch, _roll;
volatile static uint8_t _state, _isConnected, _rcConnected;

static void __capture0Handler();
static void __capture1Handler();
static void __capture2Handler();
static void __capture3Handler();
static void __pwmHandler();

void Timer_init() {
    _throttle = _yaw = _pitch = _roll = 0;
    _state = 0x00;

    //Timer0A - Throttle
    //Timer0B - Yaw
    //Timer1B - Pitch
    //Timer2A - Roll

    //Timer1A - motor0
    //Timer2B - motor1
    //Timer3A - motor2
    //Timer3B - motor3

    //Configure timers
    TimerConfigure(TIMERA0_BASE,
                   TIMER_CFG_SPLIT_PAIR |
                   TIMER_CFG_A_CAP_TIME_UP |
                   TIMER_CFG_B_CAP_TIME_UP);
    TimerConfigure(TIMERA1_BASE,
                   TIMER_CFG_SPLIT_PAIR |
                   TIMER_CFG_A_PWM |
                   TIMER_CFG_B_CAP_TIME_UP);
    TimerConfigure(TIMERA2_BASE,
                   TIMER_CFG_SPLIT_PAIR |
                   TIMER_CFG_A_CAP_TIME_UP |
                   TIMER_CFG_B_PWM);
    TimerConfigure(TIMERA3_BASE,
                   TIMER_CFG_SPLIT_PAIR |
                   TIMER_CFG_A_PWM |
                   TIMER_CFG_B_PWM);


    //Register rising edge events
    TimerControlEvent(TIMERA0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMERA0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMERA1_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
    TimerControlEvent(TIMERA2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

    //Register ISR
    TimerIntRegister(TIMERA0_BASE, TIMER_A, &__capture0Handler);
    TimerIntRegister(TIMERA0_BASE, TIMER_B, &__capture1Handler);
    TimerIntRegister(TIMERA1_BASE, TIMER_B, &__capture2Handler);
    TimerIntRegister(TIMERA2_BASE, TIMER_A, &__capture3Handler);

    //Enable capture interrupt
    TimerIntEnable(TIMERA0_BASE, TIMER_CAPA_EVENT);
    TimerIntEnable(TIMERA0_BASE, TIMER_CAPB_EVENT);
    TimerIntEnable(TIMERA1_BASE, TIMER_CAPB_EVENT);
    TimerIntEnable(TIMERA2_BASE, TIMER_CAPA_EVENT);

    //Set upper 8-bits of interval load value
    TimerPrescaleSet(TIMERA1_BASE, TIMER_A, (MOTOR_PWM_TOP) >> 16);
    TimerPrescaleSet(TIMERA2_BASE, TIMER_B, (MOTOR_PWM_TOP) >> 16);
    TimerPrescaleSet(TIMERA3_BASE, TIMER_BOTH, (MOTOR_PWM_TOP) >> 16);

    //Set lower 16-bits of interval load value
    TimerLoadSet(TIMERA1_BASE, TIMER_A, (MOTOR_PWM_TOP) & 0xFFFF);
    TimerLoadSet(TIMERA2_BASE, TIMER_B, (MOTOR_PWM_TOP) & 0xFFFF);
    TimerLoadSet(TIMERA3_BASE, TIMER_BOTH, (MOTOR_PWM_TOP) & 0xFFFF);

    //TODO: Figure out why this won't work
    //Enable interrupt for timekeeping
    //TimerControlEvent(TIMERA0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    //TimerIntRegister(TIMERA0_BASE, TIMER_A, &__pwmHandler);
    //TimerIntEnable(TIMERA0_BASE, TIMER_CAPA_EVENT | TIMER_CAPA_MATCH | TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);

    //Set motors to minimum
    Motor_set(0.f, 0.f, 0.f, 0.f);

    //Enable timers
    TimerEnable(TIMERA0_BASE, TIMER_BOTH);
    TimerEnable(TIMERA1_BASE, TIMER_BOTH);
    TimerEnable(TIMERA2_BASE, TIMER_BOTH);
    TimerEnable(TIMERA3_BASE, TIMER_BOTH);
}

void Receiver_get(struct Receiver *receiver) {
    receiver->throttle = (float)((long)_throttle - RC_PULSE_MIN) / (RC_PULSE_MAX - RC_PULSE_MIN);
    receiver->yaw = 2.f*((long)_yaw - RC_PULSE_MIN) / (RC_PULSE_MAX - RC_PULSE_MIN) - 1.f;
    receiver->pitch = 2.f*((long)_pitch - RC_PULSE_MIN) / (RC_PULSE_MAX - RC_PULSE_MIN) - 1.f;
    receiver->roll = 2.f*((long)_roll - RC_PULSE_MIN) / (RC_PULSE_MAX - RC_PULSE_MIN) - 1.f;

    receiver->throttle = CLAMP(receiver->throttle, 0.f, 1.f);
    receiver->yaw = CLAMP(receiver->yaw, -1.f, 1.f);
    receiver->pitch = CLAMP(receiver->pitch, -1.f, 1.f);
    receiver->roll = CLAMP(receiver->roll, -1.f, 1.f);

    receiver->isConnected = (_rcConnected < RC_TIMEOUT_VALUE);
}

void Motor_set(float m0, float m1, float m2, float m3) {
    //Clamp motor values
    m0 = CLAMP(m0, 0.f, 1.f);
    m1 = CLAMP(m1, 0.f, 1.f);
    m2 = CLAMP(m2, 0.f, 1.f);
    m3 = CLAMP(m3, 0.f, 1.f);

    //Motor 0
    uint32_t match = MOTOR_PWM_TOP -
            (MOTOR_MIN_PULSE + m0*(MOTOR_MAX_PULSE - MOTOR_MIN_PULSE));
    TimerPrescaleMatchSet(TIMERA1_BASE, TIMER_A, (match >> 16) & 0xFF);
    TimerMatchSet(TIMERA1_BASE, TIMER_A, match & 0xFFFF);

    //Motor 1
    match = MOTOR_PWM_TOP -
            (MOTOR_MIN_PULSE + m1*(MOTOR_MAX_PULSE - MOTOR_MIN_PULSE));
    TimerPrescaleMatchSet(TIMERA2_BASE, TIMER_B, (match >> 16) & 0xFF);
    TimerMatchSet(TIMERA2_BASE, TIMER_B, match & 0xFFFF);

    //Motor 2
    match = MOTOR_PWM_TOP -
            (MOTOR_MIN_PULSE + m2*(MOTOR_MAX_PULSE - MOTOR_MIN_PULSE));
    TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_A, (match >> 16) & 0xFF);
    TimerMatchSet(TIMERA3_BASE, TIMER_A, match & 0xFFFF);

    //Motor 3
    match = MOTOR_PWM_TOP -
            (MOTOR_MIN_PULSE + m3*(MOTOR_MAX_PULSE - MOTOR_MIN_PULSE));
    TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_B, (match >> 16) & 0xFF);
    TimerMatchSet(TIMERA3_BASE, TIMER_B, match & 0xFFFF);
}

void __pwmHandler() {
    unsigned long flags = TimerIntStatus(TIMERA3_BASE, 0);
    TimerIntClear(TIMERA0_BASE, flags);

    if(flags & TIMER_CAPA_EVENT) {
        if(_rcConnected < RC_TIMEOUT_VALUE)
            _rcConnected++;
    }
}

void __capture0Handler() {
    //Clear interrupt flags
    unsigned long flags = TimerIntStatus(TIMERA0_BASE, 0);
    TimerIntClear(TIMERA0_BASE, flags & ~(TIMER_CAPB_EVENT));

    if(flags & TIMER_CAPA_EVENT) {
        uint32_t time = //(TimerPrescaleGet(TIMERA0_BASE, TIMER_A) << 16) |
                TimerValueGet(TIMERA0_BASE, TIMER_A);

        if(_state & 0x01) {
            //Falling edge
            if(time > _throttleEdge) {
                _throttle = time - _throttleEdge;
            }
            else {
                _throttle = (time + (1 << 24)) - _throttleEdge;
            }

            //We received a pulse, reset timeout counter
            _rcConnected = 0;

            //Toggle edge detection
            TimerControlEvent(TIMERA0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
        }
        else {
            //Rising edge
            _throttleEdge = time;

            //Toggle edge detection
            TimerControlEvent(TIMERA0_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
        }

        //Toggle state for capture 0
        _state ^= 0x01;
    }
}

void __capture1Handler() {
    //Clear interrupt flags
    unsigned long flags = TimerIntStatus(TIMERA0_BASE, 0);
    TimerIntClear(TIMERA0_BASE, flags & ~(TIMER_CAPA_EVENT));

    if(flags & TIMER_CAPB_EVENT) {
        uint32_t time = TimerValueGet(TIMERA0_BASE, TIMER_B);

        if(_state & 0x02) {
            //Falling edge
            if(time > _yawEdge) {
                _yaw = time - _yawEdge;
            }
            else {
                _yaw = (time + (1 << 24)) - _yawEdge;
            }

            //Toggle edge detection
            TimerControlEvent(TIMERA0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
        }
        else {
            //Rising edge
            _yawEdge = time;

            //Toggle edge detection
            TimerControlEvent(TIMERA0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
        }

        //Toggle state for capture 1
        _state ^= 0x02;
    }
}

void __capture2Handler() {
    //Clear interrupt flags
    unsigned long flags = TimerIntStatus(TIMERA1_BASE, 0);
    TimerIntClear(TIMERA1_BASE, flags);

    if(flags & TIMER_CAPB_EVENT) {
        uint32_t time = TimerValueGet(TIMERA1_BASE, TIMER_B);

        if(_state & 0x04) {
            //Falling edge
            if(time > _pitchEdge) {
                _pitch = time - _pitchEdge;
            }
            else {
                _pitch = (time + (1 << 24)) - _pitchEdge;
            }

            //Toggle edge detection
            TimerControlEvent(TIMERA1_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
        }
        else {
            //Rising edge
            _pitchEdge = time;

            //Toggle edge detection
            TimerControlEvent(TIMERA1_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE);
        }

        //Toggle state for capture 2
        _state ^= 0x04;
    }
}

void __capture3Handler() {
    //Clear interrupt flags
    unsigned long flags = TimerIntStatus(TIMERA2_BASE, 0);
    TimerIntClear(TIMERA2_BASE, flags);

    if(flags & TIMER_CAPA_EVENT) {
        uint32_t time = TimerValueGet(TIMERA2_BASE, TIMER_A);

        if(_state & 0x08) {
            //Falling edge
            if(time > _rollEdge) {
                _roll = time - _rollEdge;
            }
            else {
                _roll = (time + (1 << 24)) - _rollEdge;
            }

            //Toggle edge detection
            TimerControlEvent(TIMERA2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
        }
        else {
            //Rising edge
            _rollEdge = time;

            //Toggle edge detection
            TimerControlEvent(TIMERA2_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
        }

        //Toggle state for capture 3
        _state ^= 0x08;
    }
}
