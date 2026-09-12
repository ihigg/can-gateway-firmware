#ifndef ICV_GATEWAY_H
#define ICV_GATEWAY_H
//--------------------------------------------------------------------------
/// \file     icv_gateway.h
/// \brief    Bus assignments, CAN IDs, message constants, timeout values
///           and prototypes for the ICV gateway.
/// \author   Isaac Higgins
/// \date     2023.05.22
//--------------------------------------------------------------------------

#include "bios_base.h" // need for uint8_t datatype
// #include "bios_can.h"
#include "os_can.h" // need for os_can_send_message

// Flags -------------------------------------------------------------------

// #define DEBUG TRUE
#define DUAL TRUE
// #define INIT TRUE
// #define ROBOTEST TRUE

// Constants ---------------------------------------------------------------

// BUS NUMBERS (relocated from the vendor config header for publication)

#define CANBUS_ID_ECU 4     // CAN_BUS_4 (CAN1) configure bus speed in MRS main screen
#define CANBUS_ID_PRIVATE 0 // CAN_BUS_0 (CAN2) configured 500kbps

// TX IDs

// IDs for initialization commands
#define INIT_CMD_ID 0xE4

// IDs for sending position to ICVs
#define CTRL_CMD_ID 0xE8

// IDs for relaying position to ECU
#define ECU_RELAY_ID 0x640 // 1600

// ID for sending error status
#define ERROR_ID 0x641 // 1601

// RX IDs

// IDs for receiving init replies from ICVs
#define ICV_INIT_ID_1 0xF5
#define ICV_INIT_ID_2 0xF6

// IDs for receiving status from ICVs
#define ICV_STATUS_ID_1 0xF9
#define ICV_STATUS_ID_2 0xFA

// ID for receiving position requests from ECU
#define ECU_POS_ID 0x5DC // 1500

// Message contents --------------------------------------------------------

// Control command byte 0 constants
#ifdef DUAL
#define CTRL_MODE_1 0x88
#define CTRL_MODE_2 0x44
#else
#define CTRL_MODE_1 0x08
#define CTRL_MODE_2 0x04
#endif // DUAL

// ECU error tx value. 1.000v = 1000 = 0x03E8
#define ECU_TX_ERROR_HIGH 0x03
#define ECU_TX_ERROR_LOW 0xE8

// Timeout values

#define WAKE_TIMEOUT_VAL 400   // 2 second
#define INIT_TIMEOUT_VAL 400   // 2 second
#define STATUS_TIMEOUT_VAL 400 // 2 second
#define ECU_TIMEOUT_VAL 400	   // 2 second

// Structs -----------------------------------------------------------------
typedef struct
{
	uint32_t arb_id;
	uint8_t b0;
	uint8_t b1;
	uint8_t b2;
	uint8_t b3;
	uint8_t b4;
	uint8_t b5;
	uint8_t b6;
	uint8_t b7;
} CANDATA8;

// Global variables --------------------------------------------------------

// FUNCTION POINTERS

// Function for sending and receiving ICV messages
extern void (*SendICV)(void);
extern void (*ReceiveICV)(bios_can_msg_typ *rx_msg);

// Function for receiving ECU messages
extern void (*ReceiveECU)(bios_can_msg_typ *rx_msg);

// Function for checking message receive timeouts
extern void (*CheckTimeout)(void);

// TIMERS AND COUNTERS

extern uint8_t init_phase;

// Function prototypes -----------------------------------------------------

// MAIN SENDING FUNCTIONS

// Initialize ICV
void init_ICV(void);

// Send normal ICV commands
void control_ICV(void);

// Send ICV information to ECU
void relay_ECU(void);

// MAIN RECEIVING FUNCTIONS

// Receive ECU position requests
void receive_ECU(bios_can_msg_typ *rx_msg);
// Helper functions for receive_ECU
bool check_ICV_system_status(bios_can_msg_typ *rx_msg);
bool check_cmd_position(uint16_t position);

// Helper function for receive_ICV_errors
void activate_error(void);

// Receive ICV initialization responses
void receive_single_ICV_init(bios_can_msg_typ *rx_msg);
void receive_dual_ICV_init(bios_can_msg_typ *rx_msg);

// ICV position message dispatchers
void receive_single_ICV_pos(bios_can_msg_typ *rx_msg);
void receive_dual_ICV_pos(bios_can_msg_typ *rx_msg);

// Store ICV position into relay message
void store_ICV_pos(bios_can_msg_typ *rx_msg, uint8_t icv);

// ERROR HANDLING

void manage_failsafe(void);

void check_wake_timeout(void);
void check_init_timeout(void);
void check_op_timeout(void);

void set_rx_error(uint8_t error_bit);
void clear_rx_error(uint8_t error_bit);
bool get_rx_error(uint8_t error_bit);

void set_tx_error(uint8_t error_bit);
void clear_tx_error(uint8_t error_bit);
bool get_tx_error(uint8_t error_bit);

// DEBUG FUNCTIONS

void robo_test(void);
uint16_t debug_cycle_position(uint16_t old_position);

// Helper function for robo_test, returns a random number between min and max, inclusive
uint16_t rand_range(uint16_t min, uint16_t max);

// Other -------------------------------------------------------------------

// BUS TRANSMIT WRAPPERS

void send_ECU_bus(CANDATA8 payload);
void send_private_bus(CANDATA8 payload);
void send_debug_packet(uint16_t arb_id, uint8_t b1);

// DO NOTHING

void receive_nothing(bios_can_msg_typ *rx_msg);
void send_nothing(void);
void timeout_nothing(void);

// CAN MESSAGE HELPER

uint8_t calc7bchk(CANDATA8 payload);

// ETC

// to get rid of compiler warning
int rand(void);

// UNUSED

// typedef struct {
//	uint32_t rxarb;
//	uint8_t rxmode;
//	uint32_t txarb;
//	uint8_t txcmd;
// } SEEKPATTERN;

#endif
