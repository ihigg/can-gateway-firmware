#ifndef WHEEL_GATEWAY_H
#define WHEEL_GATEWAY_H

void initCANC(void);
void keepAlive(void);

void bridgeTurnsWipers(bios_can_msg_typ *msg);
void bridgeColumn(bios_can_msg_typ *msg);

void translatePaddle(bios_can_msg_typ *msg);
void translateCruise(bios_can_msg_typ *msg);
void translateSwitches(bios_can_msg_typ *msg);

void sendDebugFrame(uint8_t flag);

#define CANB 4
#define CANC 0


#define PADDLE_RX_ID 0x232
#define PADDLE_TX_ID 0x501

#define CRUISE_RX_ID 0x238
#define CRUISE_TX_ID 0x500

#define TURNS_RX_ID 0x006
#define TURNS_TX_ID 0x504

#define COLUMN_RX_ID 0x296
#define COLUMN_TX_ID 0x505

#define SWITCH_RX_ID 0x1A8
#define SWITCH_TX_ID1 0x502
#define SWITCH_TX_ID2 0x503


//#define DEBUG // Comment to turn off debugging

#endif