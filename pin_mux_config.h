//*****************************************************************************
// pin_mux_config.h
//
// configure the device pins for different signals
//
// Copyright (c) 2016, Texas Instruments Incorporated - http://www.ti.com/
// All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 7/30/2016 at 3:56:43 PM
// by TI PinMux version
//
//*****************************************************************************

#ifndef __PIN_MUX_CONFIG_H__
#define __PIN_MUX_CONFIG_H__

#include <stdint.h>

#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "gpio.h"

#include "prcm.h"
#include "rom.h"
#include "rom_map.h"


#define LED_PORT	GPIOA1_BASE

#define LED_RED		0x02
#define LED_ORANGE	0x04
#define LED_GREEN	0x08

#define SW_PORT	GPIOA1_BASE
#define SW_PIN		0x20

#define GEQ_PORT		GPIOA0_BASE
#define GEQ_RESET_PIN	0x10
#define GEQ_STROBE_PIN	0x40

#define CS_CHEST_PORT	GPIOA3_BASE
#define CS_CHEST_PIN	0x10

#define CS_ARM_L_PORT	GPIOA2_BASE
#define CS_ARM_L_PIN	0x02

#define CS_ARM_R_PORT	GPIOA1_BASE
#define CS_ARM_R_PIN	0x80

#define CS_LEG_L_PORT	GPIOA0_BASE
#define CS_LEG_L_PIN	0x02

#define CS_LEG_R_PORT	GPIOA2_BASE//GPIOA3_BASE
#define CS_LEG_R_PIN	0x40//0x02

typedef enum {
	CS_CHEST = 0,
	CS_ARM_L,
	CS_ARM_R,
	CS_LEG_L,
	CS_LEG_R
} CS_e;

extern const uint32_t CS_PORT[5];
extern const uint8_t CS_PIN[5];
extern const uint8_t CS_COUNT;

extern void PinMuxConfig(void);

#endif //  __PIN_MUX_CONFIG_H__
