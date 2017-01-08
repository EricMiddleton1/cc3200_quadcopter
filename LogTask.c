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
#include "IMU.h"

#define STACK_SIZE      2048

#define PI  3.141592654f

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
    static char msg[64];
    struct Receiver receive;
    struct SensorData sensors;
    struct Angle angle;

    //Initialize the IMU
    IMU_init();

    while(1) {
        static uint8_t count = 0;
        float m0, m1, m2, m3;

        IMU_update();

        Receiver_get(&receive);
        IMU_getAngle(&angle);
        IMU_getSensors(&sensors);

        m0 = m1 = m2 = m3 = receive.throttle;

        if(receive.throttle > 0.1) {
            m0 += -0.2f * receive.pitch + 0.2f * receive.roll;
            m1 += -0.2f * receive.pitch - 0.2f * receive.roll;
            m2 += 0.2f * receive.pitch - 0.2f * receive.roll;
            m3 += 0.2f * receive.pitch + 0.2f * receive.roll;
        }

        Motor_set(m0, m1, m2, m3);

        if(!(count & 0x03)) {
            //sprintf(msg, "Gyro: %8d %8d %8d\r\n",
                    //(int)sensors.gX, (int)sensors.gY, (int)sensors.gZ);
            sprintf(msg, "Angle: %4.2f, %4.2f, %4.2f\r\n", angle.pitch*180.f/PI, angle.yaw*180.f/PI, angle.roll*180.f/PI);
            wifi_send(msg, strlen(msg));
        }

        count++;

        osi_Sleep(5);
    }
}
