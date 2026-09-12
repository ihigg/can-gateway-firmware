//--------------------------------------------------------------------------
/// \file     main_hooks.c
/// \brief    Framework entry points: init, main-loop tick, CAN RX dispatch.
/// \author   Isaac Higgins
/// \comment  Vendor example code and unused ISR stubs trimmed for publication.
//--------------------------------------------------------------------------

#include "user_code.h"
#include "wheel_gateway.h"

/// \ingroup user
/// \brief           user-defined c-code INIT
//--------------------------------------------------------------------------
/// \return          None
//--------------------------------------------------------------------------
void usercode_init(void)
{
    // Set the SW-Version, maximal length=20
    //(void)strcpy(EEPROM_SW_Version, "V1.0                ");

    //initCANC(); Works fine without this
}

/// \ingroup user
/// \brief           user-defined c-code
//--------------------------------------------------------------------------
/// \return          None
//--------------------------------------------------------------------------
void usercode(void)
{
    keepAlive();
}

/// \ingroup user
/// \brief           Manually interpret the CAN messages
//--------------------------------------------------------------------------
///                  When a CAN message is received this function will be
///                  automatically called in main-loop, while reading from
///                  the buffered messages. This routine is not called in
///                  CAN-interrupt.
///
/// \param *msg      Pointer to the message with its struct bios_can_msg_typ
/// \return          None
//--------------------------------------------------------------------------
void user_can_message_receive(uint8_t hw_id, bios_can_msg_typ *msg)
{
    (void)hw_id; // this build listens on both buses

#ifdef DEBUG
    sendDebugFrame(1);
#endif // DEBUG

    switch (msg->id)
    {
    case PADDLE_RX_ID:
        translatePaddle(msg);
        break;
    case CRUISE_RX_ID:
        translateCruise(msg);
        break;
    case SWITCH_RX_ID:
        translateSwitches(msg);
        break;
    case TURNS_RX_ID:
        bridgeTurnsWipers(msg);
        break;
    case COLUMN_RX_ID:
        bridgeColumn(msg);
        break;
    }
}

// Unused framework interrupt hooks (SCI RX/TX, 1 ms timer, user timer, ECT)
// are required by the vendor runtime but empty in this application; they are
// omitted here.
