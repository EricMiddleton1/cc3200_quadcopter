/*
 * task_wifi.h
 *
 *  Created on: Aug 8, 2016
 *      Author: eric
 */

#ifndef TASK_WIFI_H_
#define TASK_WIFI_H_

#include <stdint.h>
#include <stdlib.h>

#include "simplelink.h"

#define WIFI_ERROR_START	-1

//State enums
typedef enum {
	UNINITIALIZED,
	DISCONNECTED,
	CONNECTED_NO_IP,
	CONNECTED_WITH_IP
} APState_e;

typedef enum {
	STOPPED,
	LISTENING,
	CONNECTED
} SockState_e;

extern volatile APState_e _apState;
extern volatile SockState_e _socketState;

void wifi_init();

int wifi_start(char* ssid, char *psk);

void wifi_stop();

int wifi_send(char* buffer, int size);


#endif /* TASK_WIFI_H_ */
