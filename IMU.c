/*
 * IMU.c
 *
 *  Created on: Dec 21, 2016
 *      Author: eric
 */

#include "IMU.h"

#include <stdbool.h>

#include "hw_types.h"
#include "hw_memmap.h"
#include "i2c_if.h"
#include "task_wifi.h"

#include <math.h>

#define ADDR_GYRO           0x6B
#define ADDR_XM             0x1D

#define WHO_AM_I_G          0x0F

#define CTRL_REG1_G         0x20
#define CTRL_REG2_G         0x21
#define CTRL_REG3_G         0x22
#define CTRL_REG4_G         0x23
#define CTRL_REG5_G         0x24

#define STATUS_REG_G        0x27

#define OUT_X_L_G           0x28
#define OUT_X_H_G           0x29

#define OUT_Y_L_G           0x2A
#define OUT_Y_H_G           0x2B

#define OUT_Z_L_G           0x2C
#define OUT_Z_H_G           0x2D

#define FIFO_CTRL_REG_G     0x2E
#define FIFO_SRC_REG_G      0x2F

#define INT1_CFG_G          0x30
#define INT1_SRC_G          0x31
#define INT1_THS_XH_G       0x32
#define INT1_THS_XL_G       0x33
#define INT1_THS_YH_G       0x34
#define INT1_THS_YL_G       0x35
#define INT1_THS_ZH_G       0x36
#define INT1_THS_ZL_G       0x37
#define INT1_DURATION_G     0x38


#define OUT_TEMP_L_XM       0x05
#define OUT_TEMP_H_XM       0x06

#define STATUS_REG_M        0x07

#define OUT_X_L_M           0x08
#define OUT_X_H_M           0x09

#define OUT_Y_L_M           0x0A
#define OUT_Y_H_M           0x0B

#define OUT_Z_L_M           0x0C
#define OUT_Z_H_M           0x0D

#define WHO_AM_I_XM         0x0F

#define INT_CTRL_REG_M      0x12
#define INT_SRC_REG_M       0x13
#define INT_THS_L_M         0x14
#define INT_THS_H_M         0x15

#define OFFSET_X_L_M        0x16
#define OFFSET_X_H_M        0x17
#define OFFSET_Y_L_M        0x18
#define OFFSET_Y_H_M        0x19
#define OFFSET_Z_L_M        0x1A
#define OFFSET_Z_H_M        0x1B

#define REFERENCE_X         0x1C
#define REFERENCE_Y         0x1D
#define REFERENCE_Z         0x1E

#define CTRL_REG0_XM        0x1F
#define CTRL_REG1_XM        0x20
#define CTRL_REG2_XM        0x21
#define CTRL_REG3_XM        0x22
#define CTRL_REG4_XM        0x23
#define CTRL_REG5_XM        0x24
#define CTRL_REG6_XM        0x25
#define CTRL_REG7_XM        0x26

#define STATUS_REG_A        0x27

#define OUT_X_L_A           0x28
#define OUT_X_H_A           0x29

#define OUT_Y_L_A           0x2A
#define OUT_Y_H_A           0x2B

#define OUT_Z_L_A           0x2C
#define OUT_Z_H_A           0x2D

#define FIFO_CTRL_REG       0x2E
#define FIFO_SRC_REG        0x2F

#define INT_GEN_1_REG       0x30
#define INT_GEN_1_SRC       0x31
#define INT_GEN_1_THS       0x32
#define INT_GEN_1_DURATION  0x33
#define INT_GEN_2_REG       0x34
#define INT_GEN_2_SRC       0x35
#define INT_GEN_2_THS       0x36
#define INT_GEN_2_DURATION  0x37
#define CLICK_CFG           0x38
#define CLICK_SRC           0x39
#define CLICK_THS           0x3A
#define TIME_LIMIT          0x3B
#define TIME_LATENCY        0x3C
#define TIME_WINDOW         0x3D
#define ACT_THS             0x3E
#define ACT_DUR             0x3F


#define CPU_FREQ        80000000L

volatile static float _pitch, _yaw, _roll;
volatile static int16_t _gX, _gY, _gZ, _aX, _aY, _aZ, _mX, _mY, _mZ;

static void _writeReg(uint8_t addr, uint8_t reg, uint8_t data);
static uint8_t _readReg(uint8_t addr, uint8_t reg);

void IMU_init() {
    //Clear stored values
    _pitch = _yaw = _roll = 0.f;
    _gX = _gY = _gZ = 0;
    _aX = _aY = _aZ = 0;
    _mX = _mY = _mZ = 0;

    //Initialize I2C in master, standard speed
    I2C_IF_Open(I2C_MASTER_MODE_STD);


    //--Initialize the gyro first--//
    //760hz data rate, 100Hz bandwidth
    //Enable X, Y, Z axis
    //_writeReg(ADDR_GYRO, CTRL_REG1_G, 0xF7);

    //HPF normal mode, 0.09Hz HPF cutoff frequency
    //_writeReg(ADDR_GYRO, CTRL_REG2_G, 0x09);

    //No interrupt status pin
    //_writeReg(ADDR_GYRO, CTRL_REG3_G, 0x00);

    //500dps full-scale mode, self-test mode disabled
    //_writeReg(ADDR_GYRO, CTRL_REG4_G, 0x10);

    //HPF disabled
    //_writeReg(ADDR_GYRO, CTRL_REG5_G, 0x00);

    //--Initialize the accelerometer--//
    //400Hz update rate, all axis enabled
    _writeReg(ADDR_XM, CTRL_REG1_XM, 0x87);

    //AA filter 773Hz bandwidth, +/-8g full-scale range
    _writeReg(ADDR_XM, CTRL_REG2_XM, 0x18);

    wifi_printf("[Info] Gyro initialized. Returned whoami: %02x\r\n",
                _readReg(ADDR_GYRO, WHO_AM_I_G));
}

void IMU_update() {
    uint8_t data[6];
    uint8_t reg = OUT_X_L_A | 0x80;

    //NOTE: To read multiple bytes from the IMU, bit7 of reg must be set
    if(I2C_IF_ReadFrom(ADDR_XM, &reg, 1, data, 6) != 0) {
        char *msg = "Error reading IMU\r\n";
        wifi_send(msg, strlen(msg));
    }
    else {
        _aX = (data[1] << 8) | data[0];

        _aY = (data[3] << 8) | data[2];

        _aZ = -((data[5] << 8) | data[4]);
    }

    //Compute angle from accelerometer
    _pitch = -atan2(_aX, _aZ);
    _roll = -atan2(_aY, _aZ);
}

void IMU_getAngle(struct Angle *_angle) {
    _angle->pitch = _pitch;
    _angle->yaw = _yaw;
    _angle->roll = _roll;
}

void IMU_getSensors(struct SensorData* _sensor) {
    _sensor->gX = _gX;
    _sensor->gY = _gY;
    _sensor->gZ = _gZ;

    _sensor->aX = _aX;
    _sensor->aY = _aY;
    _sensor->aZ = _aZ;

    _sensor->mX = _mX;
    _sensor->mY = _mY;
    _sensor->mZ = _mZ;
}

static void _writeReg(uint8_t addr, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};

    if(I2C_IF_Write(addr, buf, 2, true) < 0) {
        char *msg = "Error writing IMU register\r\n";
        wifi_send(msg, strlen(msg));
    }
}

static uint8_t _readReg(uint8_t addr, uint8_t reg) {
    uint8_t data = 0x00;

    if(I2C_IF_ReadFrom(addr, &reg, 1, &data, 1) < 0) {
        char *msg = "Error reading from IMU register\r\n";
        wifi_send(msg, strlen(msg));
    }

    return data;
}
