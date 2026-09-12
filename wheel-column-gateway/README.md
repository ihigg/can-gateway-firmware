# Steering Column / Wheel Gateway

Lets a steering column switch module from one vehicle (turn signals, wipers, cruise control, shift paddles, column tilt and telescope) work in a car whose body electronics expect completely different CAN messages. Neither side could be changed, so the gateway listens on CAN B, translates, and sends the result on CAN C.

## Files

- `main_hooks.c`: init, main loop, and CAN receive dispatch
- `wheel_gateway.h`: bus numbers, CAN IDs, and the debug flag
- `wheel_gateway.c`: keep-alive and the translation for each signal

## How it works

The column module won't send anything unless it thinks the car is awake. `keepAlive()` sends a wake frame on CAN B every 500 ms and a keep-alive on CAN C every 100 ms. It uses `os_time_past()` timestamp checks instead of delays so it never blocks the main loop. Getting the wake sequence and timing right was most of the debugging on this project. `initCANC()` sends five wake frames at boot. It turned out to be unnecessary once the keep-alive was right, so I left it in but disabled.

`user_can_message_receive()` uses a switch on the CAN ID to pick the right handler. Where the bits already line up, the handler just forwards the data under a new CAN ID (`bridgeTurnsWipers()`, `bridgeColumn()`). The receiving system reads some signals as 16-bit words though, and the column module packs up to eight switches into one byte. For those, `translatePaddle()` and `translateSwitches()` mask out each bit, shift it down, and put it in its own word. The switch panel has more signals than fit in one frame, so `translateSwitches()` splits them across two IDs.

Each handler returns early if none of its bits are set, so the gateway only sends a frame when a control is actually active. This keeps unnecessary traffic off the bus.

With `DEBUG` defined in `wheel_gateway.h`, `sendDebugFrame(flag)` sends a frame on `0x7F0 + flag`, so each code path shows up as its own ID on a CAN analyzer. There was no debugger on the module, so this is how I followed what the code was doing.

## CAN IDs

| Signal | RX (CAN B) | TX (CAN C) |
|---|---|---|
| Shift paddles | `0x232` | `0x501` |
| Cruise control | `0x238` | `0x500` |
| Switch panel | `0x1A8` | `0x502`, `0x503` |
| Turn signals / wipers | `0x006` | `0x504` |
| Column tilt / telescope | `0x296` | `0x505` |
