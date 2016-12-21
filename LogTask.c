/*
 * LogTask.c
 *
 *  Created on: Dec 20, 2016
 *      Author: eric
 */

#include "LogTask.h"
#include "task_wifi.h"
#include <stdlib.h>
#include <string.h>
#include "Timer.h"

#define STACK_SIZE      2048

static void __logTask();

void LogTask_init() {
    int retval = osi_TaskCreate(&__logTask,
            (const signed char*)"Log Task",
            STACK_SIZE,
            NULL,
            1,
            NULL);
}

void __logTask() {
    char msg[64];
    struct Receiver receive;

    while(1) {
        static uint8_t count = 0;
        Receiver_get(&receive);

        float motorValue = receive.throttle;
        if(motorValue > 0.1)
            motorValue += receive.pitch / 5;

        Motor_set(motorValue, motorValue, motorValue, motorValue);

        if(receive.isConnected) {
            //sprintf(msg, "%3.2f %3.2f %3.2f %3.2f\r\n",
                    //receive.throttle, receive.yaw, receive.pitch, receive.roll);
            sprintf(msg, "Motor value: %2.2f\r\n", motorValue);
        }
        else {
            strcpy(msg, "Not connected");
        }
        wifi_send(msg, strlen(msg));

        count++;

        osi_Sleep(50);
    }
}
