/*
 * IMU.h
 *
 *  Created on: Dec 21, 2016
 *      Author: eric
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

struct Angle {
    float pitch, yaw, roll;
};

struct SensorData {
    int16_t gX, gY, gZ;
    int16_t aX, aY, aZ;
    int16_t mX, mY, mZ;
};

void IMU_init();

void IMU_update();

void IMU_getAngle(struct Angle*);

void IMU_getSensors(struct SensorData*);


#endif /* IMU_H_ */
