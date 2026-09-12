//--------------------------------------------------------------------------
/// \file     main_hooks.c
/// \brief    Framework entry points: boot init, 5 ms main-loop tick,
///           CAN RX dispatch by bus.
/// \author   Isaac Higgins
/// \date     2023.05.22
/// \comment  Unused vendor interrupt stubs trimmed for publication.
//--------------------------------------------------------------------------

// #include <sys/timeb.h>

#define GRAPH_DISABLE

// #define NO_STDIO_REDIRECT

#include "user_code.h"		// MRS required parameters
#include "icv_gateway.h"   // All our custom vars/globals

// Function variables
void (*SendICV)(void);
void (*ReceiveICV)(bios_can_msg_typ *);
void (*ReceiveECU)(bios_can_msg_typ *);
void (*CheckTimeout)(void);

// Timers and Counters
uint32_t time_val;

// --------------------------------------------------------------------------------
// User Variables
// --------------------------------------------------------------------------------

// Assumption: This is inserted into MRS init and is called once at startup
void usercode_init(void)
{
#ifdef DEBUG
	(void)send_debug_packet(1000, 0x0A);
#endif

	SendICV = init_ICV;
	ReceiveECU = receive_ECU;
	CheckTimeout = check_wake_timeout;
#ifdef DUAL
	ReceiveICV = receive_dual_ICV_init;
#else
	ReceiveICV = receive_single_ICV_init;
#endif // DUAL
}

// This is inserted into main loop of MRS code and is called each time
void usercode(void)
{
	// 10ms is default configuration
	// must set uint16_t graph_cycle_time = 1; in graph_code.c

	// Send command packets every 5ms
	if (os_time_past(time_val, 5, OS_1ms))
	{
#ifdef DEBUG
		//(void)send_debug_packet(1100, 0x00);
#endif

		// if the time passed, set "time_val" to a new timestamp to start cycling
		os_timestamp(&time_val, OS_1ms);

		// TODO check the order of these
		SendICV();
		relay_ECU();
		CheckTimeout();
		manage_failsafe();
	}
}

// Unused framework interrupt hooks (1 ms timer, user timer, SCI RX/TX, ECT)
// are required by the vendor runtime but empty in this application; they are
// omitted here.
