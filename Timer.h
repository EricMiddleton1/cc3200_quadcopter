/*
 * Receiver.h
 *
 *  Created on: Dec 19, 2016
 *      Author: eric
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

struct Receiver {
    float throttle;
    float yaw;
    float pitch;
    float roll;

    uint8_t isConnected;
};

void Timer_init();

void Receiver_get(struct Receiver*);

void Motor_set(float m0, float m1, float m2, float m3);

#endif /* TIMER_H_ */
