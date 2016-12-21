/*
 * cc3200_quadcopter
 *
 * Written by Eric Middleton
 */

// Simplelink includes
#include "simplelink.h"

// driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "gpio.h"
#include "Console.h"
#include "LogTask.h"

// free_rtos/ti-rtos includes
#include "osi.h"

// common interface includes
#include "common.h"

#include "Timer.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

#define OSI_STACK_SIZE          2048

#define DEFAULT_BAUD			115200

#include "pin_mux_config.h"

//Custom tasks
#include "task_wifi.h"

//*****************************************************************************
//
//! Board Initialization & Configuration
//! Note: Taken from wlan_ap example program
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs) || defined(gcc)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

/*
 *  ======== main ========
 */
int main(void)
{
	//Board initialization
	BoardInit();

    //Initialize the pin configuration
    PinMuxConfig();

    //Initialize the wifi system
    wifi_init();

    Timer_init();

    LogTask_init();

    int retval;

    //Start simplelink host
    retval = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if(retval < 0) {
    	for(;;);
    }

    //Start task scheduler
    osi_start();

    return 0;
}
