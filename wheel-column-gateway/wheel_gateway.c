#include "user_code.h"
#include "wheel_gateway.h"

uint32 time_val;
uint32 time_val2;

// CAN B must be woken first. Send 5 wakeup messages before main loop
void initCANC(void)
{
    int i;
    for (i = 0; i < 5; i++)
    {
        os_can_send_msg(CANB, 0x000, 0, 6, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00); // Nopt sure if I need all 8 filled or not
        os_wait(500);
    }
}

// Send regular frames required for column functionality
void keepAlive(void)
{
    // CAN B keep alive
    if (os_time_past(time_val, 500, OS_1ms))
    {
        // if the time passed, set "time_val" to a new timestamp to start cycling
        os_timestamp(&time_val, OS_1ms);
        os_can_send_msg(CANB, 0x000, 0, 6, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00);
    }

    // CAN C keep alive
    if (os_time_past(time_val2, 100, OS_1ms))
    {
        os_timestamp(&time_val2, OS_1ms);
        os_can_send_msg(CANC, 0x210, 0, 4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    }
}

// Paddle frames can only be interpreted as words, shift data bits into their own words and broadcast on CANC
void translatePaddle(bios_can_msg_typ *msg)
{
#ifdef DEBUG
    sendDebugFrame(2);
#endif // DEBUG

if((msg->data[0] & 3) > 0) { // Bitmask 00001111
        os_can_send_msg(CANC,
                        PADDLE_TX_ID,
                        0,
                        4,
                        0x00,
                        (msg->data[0] & 2) >> 1, // Bitmask 00000010 on first byte, transmit on first bit of second byte (first word)
                        0x00,
                        (msg->data[0] & 1), // Bitmask 00000001, no need to shift just set to fourth byte (second word)
                        0x00, 0x00, 0x00, 0x00);
}
}

// Cruise frames can only be interpreted as words, shift data bits into their own words and broadcast on CANC
void translateCruise(bios_can_msg_typ *msg)
{
#ifdef DEBUG
    sendDebugFrame(3);
#endif // DEBUG

   if((msg->data[0] & 15) > 0) {
        os_can_send_message(CANC,
                            CRUISE_TX_ID,
                            0,
                            8,
                            0x00,
                            (msg->data[0] & 8) >> 3, // Bitmask 00001000
                            0x00,
                            (msg->data[0] & 4) >> 2, // Bitmask 00000100
                            0x00,
                            (msg->data[0] & 2) >> 1, // Bitmask 00000010
                            0x00,
                            (msg->data[0] & 1)); // Bitmask 00000001
   }
}

// Switch frames can only be interpreted as words, we need two frames to hold all the flags
// shift data bits into their own words and broadcast on CANC
void translateSwitches(bios_can_msg_typ *msg)
{
#ifdef DEBUG
    sendDebugFrame(4);
#endif // DEBUG

    if ((msg->data[0] & 240) > 0) // Is there anything in the last 4 bits?
    {
#ifdef DEBUG
        sendDebugFrame(6);
#endif // DEBUG

        // Second frame:
        os_can_send_msg(CANC,
                        SWITCH_TX_ID1,
                        0,
                        8,
                        0x00,
                        (msg->data[0] & 128) >> 7,// Bitmask 10000000
                        0x00,
                        (msg->data[0] & 64) >> 6, // Bitmask 01000000
                        0x00,
                        (msg->data[0] & 32) >> 5, // Bitmask 00100000
                        0x00,
                        (msg->data[0] & 16) >> 4);// Bitmask 00010000
    }

    if ((msg->data[0] & 15) > 0) // Is there anything in the first 4 bits?
    {
#ifdef DEBUG
        sendDebugFrame(5);
#endif // DEBUG

        // First frame:
        os_can_send_msg(CANC,
                        SWITCH_TX_ID2,
                        0,
                        8,
                        0x00,
                        (msg->data[0] & 8) >> 3,// Bitmask 00001000
                        0x00,
                        (msg->data[0] & 4) >> 2,// Bitmask 00000100
                        0x00,
                        (msg->data[0] & 2) >> 1,// Bitmask 00000010
                        0x00,
                        (msg->data[0] & 1));    // Bitmask 00000001
    }
}

// Rebroadcast turns and wipers signals to CANC, no need to shift anything
void bridgeTurnsWipers(bios_can_msg_typ *msg)
{
#ifdef DEBUG
    sendDebugFrame(7);
#endif // DEBUG

    if((msg->data[0] & 255) > 0 || (msg->data[1] & 252) > 0) { // Bitmask 11111111 and 11111100
        os_can_send_msg(CANC, TURNS_TX_ID, 0, 2, msg->data[0], msg->data[1], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    }
}

// Rebroadcast Column tilt/tele signals to CANC, no need to shift anything
void bridgeColumn(bios_can_msg_typ *msg)
{
#ifdef DEBUG
    sendDebugFrame(8);
#endif // DEBUG

    if((msg->data[0] & 252) > 0) { // Bitmask 11111100
        os_can_send_msg(CANC, COLUMN_TX_ID, 0, 1, msg->data[0], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    }
}

// For transmitting debug information on the CAN bus
void sendDebugFrame(uint8_t flag)
{
    os_can_send_msg(CANC, 2032 + flag, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0); // Sends a message with ID 0x7F(flag)
}