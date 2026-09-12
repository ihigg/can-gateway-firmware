//--------------------------------------------------------------------------
/// \file     icv_gateway.c
/// \brief    ICV gateway protocol implementation: init handshake, position
///           command assembly, status decode, error flags and failsafe.
/// \author   Isaac Higgins
/// \date     2023.05.22
//--------------------------------------------------------------------------

#ifndef ICV_GATEWAY_H
#include "icv_gateway.h"
#endif

// Core command variables
uint16_t icv_cmd_position = 0;
uint8_t init_phase = 1;

// ECU relay variables
CANDATA8 relay_msg =
	{
		ECU_RELAY_ID,
		0x0, // ICV 1 cmd position relay low byte
		0x0, // ICV 1 cmd position relay high byte
		0x0, // ICV 1 actual position low byte
		0x0, // ICV 1 actual position high byte
		0x0, // ICV 2 cmd position relay low byte
		0x0, // ICV 2 cmd position relay high byte
		0x0, // ICV 2 actual position low byte
		0x0	 // ICV 2 actual position high byte
};

CANDATA8 error_msg =
	{
		ERROR_ID,
		0x0, // Rx error bit field
		0x0, // Tx error bit field
		0x0, // ICV 1 system status from ECU
		0x0, // ICV 2 system status from ECU
		0x0,
		0x0,
		0x0,
		0x0};
/**
	Error Code format. 1 is error, 0 is no error. Message timeouts are 2 seconds:

	Rx error bit field:
	Bit 0: ICV 1 Wake response timeout // TODO should we have seperate timeouts for each ICV afterall?
	Bit 1: ICV 2 Wake response timeout
	Bit 2: Init response timeout (if init is enabled)
	Bit 3: ECU request timeout
	Bit 4: ICV 1 status (pos reply) timeout
	Bit 5: ICV 2 status (pos reply) timeout
	Bit 6: Bad ECU position - position out of range
	Bit 7: System status fault from ECU

	Tx error bit field:
	Bit 0: MRS failsafe status - Set if MRS goes into 30% throttle position failsafe
	Bit 1: ECU bus TX fail
	Bit 2: Private bus TX fail
	Bit 3: ECU Bus Heavy // TODO check if this is possible
	Bit 4: Private Bus Heavy // TODO check if this is possible
*/

// Timeout variables

uint32_t init_timeout = 0;

uint32_t wake_timeout_1 = 0;
bool icv_1_awake = FALSE;

uint32_t status_timeout_1 = 0;

uint32_t ecu_timeout = 0;

uint32_t wake_timeout_2 = 0;
bool icv_2_awake = FALSE;

uint32_t status_timeout_2 = 0;

// -----------------------------------------------------------------------------
// ECU receive handlers
// -----------------------------------------------------------------------------

// Receive ECU position requests
void receive_ECU(bios_can_msg_typ *rx_msg)
{
	uint16_t position = (rx_msg->data[1] << 8) + rx_msg->data[0]; // Grab 10bit number from two bytes

	// Guard clause
	if (rx_msg->id != ECU_POS_ID)
	{
		return;
	}

#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

	ecu_timeout = 0;
	clear_rx_error(3);

	// Check if ECU is good to run
	if (check_ICV_system_status(rx_msg) && check_cmd_position(position))
	{
		// If MRS is in failsafe, don't listen to ECU position requests
		// manage_failsafe() has set the position to 30% already
		// (This is nested because we want to check the system status and position every time)
		if (get_tx_error(0))
		{
			return;
		}

		// If not in failsafe, store position from ECU
		icv_cmd_position = position;
	}
}

// Helper functions for receive_ECU

// Returns true if ECU says ICV system is good to run
// Also sets system status relay data in error_msg (because its convenient to do it here)
bool check_ICV_system_status(bios_can_msg_typ *rx_msg)
{
	// Store system status relay data
	error_msg.b2 = rx_msg->data[2];
	error_msg.b3 = rx_msg->data[4]; // Probably fine to not wrap this in ifdef DUAL

	// Check if error status matches any good status
	if (rx_msg->data[2] != 0x01 && rx_msg->data[2] != 0x0A && rx_msg->data[2] != 0x0F
#ifdef DUAL
		|| rx_msg->data[4] != 0x01 && rx_msg->data[4] != 0x0A && rx_msg->data[4] != 0x0F
#endif // DUAL
	)
	{
		set_rx_error(7);
		return FALSE;
	}
	else
	{
		clear_rx_error(7);
		return TRUE;
	}
}

// Returns true if position is within valid range
bool check_cmd_position(uint16_t position)
{
	if (position < 0 || position > 0x3E8) // Bad position
	{
		set_rx_error(6);
		return FALSE;
	}
	else // Good position
	{
		clear_rx_error(6);
		return TRUE;
	}
}

// -----------------------------------------------------------------------------
// ICV receive handlers
// -----------------------------------------------------------------------------

// TODO - should we use defines for the data[0] values?
// TODO we can combine these by wrapping 'else if (rx_msg->data[0] == 0x25)...' in an ifdef in receive_dual_ICV_init

// Receive ICV initialization responses
void receive_single_ICV_init(bios_can_msg_typ *rx_msg)
{
	if (rx_msg->id != ICV_INIT_ID_1)
	{
		return;
	}

#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

	if (init_phase == 1 && rx_msg->data[0] == 0x15) // ICV wakeup response packet
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
	   // no clear_rx_error() because we don't want to clear the error if we get a wakeup response after the timeout
		wake_timeout_1 = 0;
		icv_1_awake = TRUE;
		++init_phase;
	}
	else if (init_phase == 2 && rx_msg->data[0] == 0x06) // ICV ACK-ACK packet
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
		init_timeout = 0;
		clear_rx_error(2);
		++init_phase;
	}
}

void receive_dual_ICV_init(bios_can_msg_typ *rx_msg)
{
	static uint32_t last_reply = 0;

	if (rx_msg->id != ICV_INIT_ID_1 && rx_msg->id != ICV_INIT_ID_2)
	{
		return;
	}

#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

	if (init_phase == 1)
	{
		if (rx_msg->data[0] == 0x15) // ICV 1
		{
#ifdef DEBUG
			(void)send_debug_packet(__LINE__, 0x00);
#endif							// DEBUG
			wake_timeout_1 = 0; //? Needed?
			icv_1_awake = TRUE;
		}
		else if (rx_msg->data[0] == 0x25) // ICV 2
		{
#ifdef DEBUG
			(void)send_debug_packet(__LINE__, 0x00);
#endif							// DEBUG
			wake_timeout_2 = 0; //? Needed?
			icv_2_awake = TRUE;
		}

		if (icv_1_awake && icv_2_awake)
		{
#ifdef DEBUG
			(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
			++init_phase;
		}
	}
	else if (init_phase == 2 && rx_msg->data[0] == 0x06) // We are only expecting ICV 1 to respond, because we don't know how to prompt ICV 2
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif					  // DEBUG
		init_timeout = 0; //? Needed?
		clear_rx_error(2);
		++init_phase;
	}
}

// TODO we can combine these similiarly to receive_dual_ICV_init

// Receive ICV position status messages
void receive_single_ICV_pos(bios_can_msg_typ *rx_msg)
{
	if (rx_msg->id == ICV_STATUS_ID_1)
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

		status_timeout_1 = 0;
		clear_rx_error(4);

		store_ICV_pos(rx_msg, 1);
	}
}

void receive_dual_ICV_pos(bios_can_msg_typ *rx_msg)
{
	if (rx_msg->id == ICV_STATUS_ID_1)
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

		status_timeout_1 = 0;
		clear_rx_error(4);

		store_ICV_pos(rx_msg, 1);
	}
	else if (rx_msg->id == ICV_STATUS_ID_2)
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

		status_timeout_2 = 0;
		clear_rx_error(5);

		store_ICV_pos(rx_msg, 2);
	}
}

// Helper function to store ICV position into relay message
// arg icv is the ICV number (1 or 2)
void store_ICV_pos(bios_can_msg_typ *rx_msg, uint8_t icv)
{
#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

	if (icv == 1)
	{
		// First position is a relay of the command the ICV received
		relay_msg.b0 = rx_msg->data[3];
		relay_msg.b1 = rx_msg->data[4] & 0x3; // Mask off high byte4

		// Second position is the actual position of the ICV
		relay_msg.b2 = ((rx_msg->data[5] & 0xF) << 4) + ((rx_msg->data[4] & 0xF0) >> 4);
		relay_msg.b3 = (rx_msg->data[5] & 0x30) >> 4; // Mask off high byte and move right
	}
	else if (icv == 2)
	{
		relay_msg.b4 = rx_msg->data[3];
		relay_msg.b5 = rx_msg->data[4] & 0x3; // Mask off high byte

		relay_msg.b6 = ((rx_msg->data[5] & 0xF) << 4) + ((rx_msg->data[4] & 0xF0) >> 4);
		relay_msg.b7 = (rx_msg->data[5] & 0x30) >> 4; // Mask off high byte and move right
	}

	// relay_ECU will transmit relay_msg in the 5ms loop
}

// TODO Delete this function
// Helper function for receive_ICV_errors
void activate_error(void)
{
	relay_msg.b6 = ECU_TX_ERROR_HIGH;
	relay_msg.b7 = ECU_TX_ERROR_LOW;
}

// -----------------------------------------------------------------------------
// ECU transmit handlers
// -----------------------------------------------------------------------------

// Send ICV information to ECU
void relay_ECU(void)
{
#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
	send_private_bus(relay_msg);
#endif // DEBUG

	send_ECU_bus(relay_msg);
	send_ECU_bus(error_msg);
}

// -----------------------------------------------------------------------------
// ICV transmit handlers
// -----------------------------------------------------------------------------

void init_ICV(void)
{
	// General variables
	static uint8_t loop_counter = 1;

	static CANDATA8 icv_cmd =
		{
			INIT_CMD_ID, // arb_id
			0x55,		 // b1: init mode?
			0xFF,		 // b2:
			0xFF,		 // b3:
			0xFF,		 // b4:
			0xFF,		 // b5:
			0xFF,		 // b6:
			0xFF,		 // b7:
			0x55		 // b8: XOR checksum B1-B7
		};

#ifdef DEBUG
	(void)send_debug_packet(__LINE__, init_phase);
#endif // DEBUG

	// handle init phases w arb_id 0xE4
	//--------------------------

	switch (init_phase)
	{
	case 1: // send wakeup packets, we do this even if INIT flag is not set
		send_private_bus(icv_cmd);
		break;

	case 2: // send wakeup ACK packet, or skip init sequence if INIT flag is not set
#ifdef INIT
		icv_cmd.b0 = 0x06;
		icv_cmd.b1 = 0x01;
		icv_cmd.b7 = calc7bchk(icv_cmd);
		send_private_bus(icv_cmd);

		CheckTimeout = check_init_timeout;
		break;
#else
		init_phase = 5;		   // Fall through to end of init sequence
#endif // INIT

	case 3: // pause for 3 intervals
		if (loop_counter > 3)
		{
			loop_counter = 0;
			++init_phase;
		}
		else
		{
			++loop_counter;
		}
		break;

	case 4:
		++init_phase; // fall through to...//? Why do we need this?

	default:
#ifdef ROBOTEST
		SendICV = robo_test;
		ReceiveECU = receive_nothing;
		CheckTimeout = timeout_nothing;
		// TODO we should still check for ICV status messages
#else  // NOT ROBOTEST
		SendICV = control_ICV; // bypass init section from now on
		CheckTimeout = check_op_timeout;
#endif // ROBOTEST

#ifdef DUAL
		ReceiveICV = receive_dual_ICV_pos;
#else  // NOT DUAL
		ReceiveICV = receive_single_ICV_pos;
#endif // DUAL
	}
}

// Send icv position commands
void control_ICV(void)
{
	// General variables
	static uint8_t loop_counter = 1;
	static uint16_t lastposition = 0;

	static CANDATA8 icv_cmd =
		{
			CTRL_CMD_ID, // arb_id
			0x08,		 // b1: mode command?
			0x00,		 // b2: hi = low position bits
			0x08,		 // b3: 7&8 = high position bits, 4th bit always set
			0x00,		 // b4:
			0x08,		 // b5:
			0xFF,		 // b6:
			0x6F,		 // b7: Hi = counter, Lo = 0xF
			0x98		 // b8: checksum B1-7
		};

#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

	// Normal Business w/ arb_id 0xE8
	//--------------------------

	// set the position for ICV 1
	icv_cmd.b2 = (icv_cmd_position >> 8) | 0x08; // get the high byte for bits 1-2, and set the 4th bit always
	icv_cmd.b1 = icv_cmd_position & 0x00FF;		 // clear hi byte keep low (prob unecessary)

#ifdef DUAL
	// set the position for ICV 2
	icv_cmd.b4 = (icv_cmd_position >> 8) | 0x08; // get the high byte for bits 1-2, and set the 4th bit always
	icv_cmd.b3 = icv_cmd_position & 0x00FF;		 // clear hi byte keep low (prob unecessary)
#endif											 // DUAL

	// calc the check byte just before sending
	icv_cmd.b7 = calc7bchk(icv_cmd);

	// Send command to ICV
	send_private_bus(icv_cmd);

	// TODO - this might not be working, during a previous run it never went back to 0x08
	// Send 3 0x04 packets if the position goes > 0
	// factory sends one packet before / without the mode switch which is why comes after the send
	if (init_phase == 6)
	{
		if (loop_counter > 3)
		{
			// we sent 3 0x04 packets, go back to 0x08
			icv_cmd.b0 = CTRL_MODE_1;
			loop_counter = 0;
			--init_phase;
		}
		else
		{
			++loop_counter;
		}
	}
	else if (icv_cmd_position > 0 && lastposition == 0)
	{
		++init_phase; // next time go to 6 and send the 0x04's
		icv_cmd.b0 = CTRL_MODE_2;
		++loop_counter;
	}

	// Setup for next send
	icv_cmd.b6 = icv_cmd.b6 + 0x10; // diddle the counter add to high nibble only, leave low nibble
	lastposition = icv_cmd_position;
}

// -----------------------------------------------------------------------------
// Error handling
// -----------------------------------------------------------------------------

/// @brief Checks error flags. If appropriate, sets position to 30%, sets failsafe flag and respective timeout flag.
void manage_failsafe(void)
{
	// True if any rx error except wake or init
	// Tx errors don't trigger failsafe.
	bool critical_error = (error_msg.b0 & 0xF8) > 0;

#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

	if (!get_tx_error(0) && critical_error) // If failsafe is not already set and should be
	{
		// Set to 30%
		icv_cmd_position = 0x12C;

		// Set failsafe flag, which receive_ECU will see and bypass ECU requests
		set_tx_error(0);
	}
	else if (get_tx_error(0) && !critical_error) // If failsafe is set and all errors are cleared
	{
		clear_tx_error(0);
	}
}

// Sets/clears the error flag at the given bit
void set_rx_error(uint8_t error_bit)
{
	error_msg.b0 |= (1 << error_bit);
}

void set_tx_error(uint8_t error_bit)
{
	error_msg.b1 |= (1 << error_bit);
}

void clear_rx_error(uint8_t error_bit)
{
	error_msg.b0 &= ~(1 << error_bit);
}

void clear_tx_error(uint8_t error_bit)
{
	error_msg.b1 &= ~(1 << error_bit);
}

bool get_rx_error(uint8_t error_bit)
{
	// Mask for given bit and shift
	return (error_msg.b0 & (1 << error_bit)) >> error_bit;
}

bool get_tx_error(uint8_t error_bit)
{
	// Mask for given bit and shift
	return (error_msg.b1 & (1 << error_bit)) >> error_bit;
}

// Timeout checks ---------------------------------------------------------------

// Timers are reset in the receive functions

void check_wake_timeout(void)
{
#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
	if (wake_timeout_1 > WAKE_TIMEOUT_VAL)
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
		set_rx_error(0);

		// Prevent further send/receive
		SendICV = send_nothing;
		ReceiveICV = receive_nothing;
	}
	else if (!icv_1_awake) // icv_1_awake is set in receive_*_icv_init
	{
		wake_timeout_1++;
	}

#ifdef DUAL
	if (wake_timeout_2 > WAKE_TIMEOUT_VAL)
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
		set_rx_error(1);

		SendICV = send_nothing;
		ReceiveICV = receive_nothing;
	}
	else if (!icv_2_awake)
	{
		wake_timeout_2++;
	}
#endif // DUAL
}

void check_init_timeout(void)
{
#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
	if (init_timeout > INIT_TIMEOUT_VAL)
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
		set_rx_error(2);
	}
	else
	{
		init_timeout++;
	}
}

// Status and ECU timeouts
void check_op_timeout(void)
{
#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
	if (ecu_timeout > ECU_TIMEOUT_VAL)
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
		set_rx_error(3);
	}
	else
	{
		ecu_timeout++;
	}

	if (status_timeout_1 > STATUS_TIMEOUT_VAL)
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
		set_rx_error(4);
	}
	else
	{
		status_timeout_1++;
	}

#ifdef DUAL
	if (status_timeout_2 > STATUS_TIMEOUT_VAL)
	{
#ifdef DEBUG
		(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
		set_rx_error(5);
	}
	else
	{
		status_timeout_2++;
	}
#endif // DUAL
}

// -----------------------------------------------------------------------------
// Robotest
// -----------------------------------------------------------------------------

void robo_test(void)
{
#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG
	icv_cmd_position = debug_cycle_position(icv_cmd_position);
	control_ICV();
}

/// @brief Debug function for sweeping position
/// @details Sweeps position from 0 to 1000 and back.
///	Occasionally changes speed.
///	Occasionally jumps to a random position.
///	Occasionally holds a position for a random number of packets.
/// @param old_position the last position sent to the ICV
/// @return the new position to send to the ICV
uint16_t debug_cycle_position(uint16_t old_position)
{
	static uint16_t hold_wait = 0;
	static uint16_t hold_counter = 0;
	static uint16_t jump_wait = 0;
	static uint16_t speed_wait = 0;
	static uint8_t speed = 1;
	static bool down = FALSE;

	uint16_t position = 0;

#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

	// Reminder: Rate is 1000/5 = 200 packets per second

	// Wait 1 to 10 seconds and then hold for 1 to 5 seconds
	if (hold_wait > 0) // If we're waiting to hold, decrement the counter
	{
		--hold_wait;
	}
	else if (hold_counter > 0) // Otherwise, we're done waiting. If we're holding, decrement the counter and return the old position
	{
		--hold_counter;
		return old_position;
	}
	else // Otherwise, we're done holding, so set up the next hold
	{
		// Random wait 1 to 10 seconds
		hold_wait = rand_range(1, 10);
		// Random hold 1 to 5 seconds
		hold_counter = rand_range(1, 5);
	}

	// Wait 5 to 10 seconds, then jump to a random position
	if (jump_wait > 0) // If we're waiting to jump, decrement the counter
	{
		--jump_wait;
	}
	else // Otherwise, we're done waiting, so jump to a random position and set up the next jump
	{
		// Random wait 1 to 10 seconds
		jump_wait = rand_range(1, 10);

		position = rand() % 1001;

		return position;
	}

	// Change speed every 1 to 5 seconds
	if (speed_wait > 0) // If we're waiting to change speed, decrement the counter
	{
		--speed_wait;
	}
	else // Otherwise, we're done waiting, so change speed and set up the next speed change
	{
		// Random wait 1 to 5 seconds
		speed_wait = rand_range(1, 5);

		// Random speed 1 to 10
		speed = rand() % 10 + 1;
	}

	// Bump position in the direction we are going and dont overflow
	if (down)
	{
		if (speed > old_position)
		{
			position = 0;
			down = FALSE;
		}
		else
		{
			position = old_position - speed;
		}
	}
	else
	{
		if (old_position + speed >= 1000)
		{
			position = 1000;
			down = TRUE;
		}
		else
		{
			position = old_position + speed;
		}
	}
	return position;
}

/// @brief Generate a random duration within the given range
/// @param min the minimum value in seconds
/// @param max the maximum value in seconds
/// @return a random duration between min and max, inclusive, in packets
uint16_t rand_range(uint16_t min, uint16_t max)
{
	// Packet rate is 200 packets per second
	uint16_t min_packets = min * 200;
	uint16_t max_packets = max * 200;

#ifdef DEBUG
	(void)send_debug_packet(__LINE__, 0x00);
#endif // DEBUG

	return rand() % (max_packets - min_packets + 1) + min_packets;
}

// -----------------------------------------------------------------------------
// Send wrappers
// -----------------------------------------------------------------------------

// Send wrapper for Emtron ECU bus
void send_ECU_bus(CANDATA8 payload)
{
	if (os_can_send_message( // 0=OK, 1=Error(Send buffer full or CAN not initialized or bus heavy)
			CANBUS_ID_ECU,
			payload.arb_id,
			0, 8, // not extended, 8bytes
			payload.b0,
			payload.b1,
			payload.b2,
			payload.b3,
			payload.b4,
			payload.b5,
			payload.b6,
			payload.b7))
	{
		set_tx_error(1);
	}
	else
	{
		clear_tx_error(1);
	}
}

// Send wrapper for BMW private throttles bus
void send_private_bus(CANDATA8 payload)
{
	if (os_can_send_message(
			CANBUS_ID_PRIVATE,
			payload.arb_id,
			0, 8, // not extended, 8bytes
			payload.b0,
			payload.b1,
			payload.b2,
			payload.b3,
			payload.b4,
			payload.b5,
			payload.b6,
			payload.b7))
	{
		set_tx_error(2);
	}
	else
	{
		clear_tx_error(2);
	}
}

void send_debug_packet(uint16_t arb_id, uint8_t b1)
{
	/**
			(void)os_can_send_message(
				CANBUS_ID_PRIVATE, // change if debugs to send on different network
				arb_id,
				0, 8, // not extended, 8bytes
				b1,
				0x22,
				0x33,
				0x44,
				0x55,
				0x66,
				0x77,
				0x88);
	*/

	(void)os_can_send_message(
		CANBUS_ID_ECU, // change if debugs to send on different network
		arb_id,
		0, 8, // not extended, 8bytes
		b1,
		0x22,
		0x33,
		0x44,
		0x55,
		0x66,
		0x77,
		0x88);
}

// -----------------------------------------------------------------------------
// Do nothing functions
// -----------------------------------------------------------------------------

void receive_nothing(bios_can_msg_typ *rx_msg) {}

void send_nothing(void) {}

void timeout_nothing(void) {}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

// Xor bytes 1-7 for a payload
uint8_t calc7bchk(CANDATA8 payload)
{
	return payload.b0 ^ payload.b1 ^ payload.b2 ^ payload.b3 ^ payload.b4 ^ payload.b5 ^ payload.b6;
}