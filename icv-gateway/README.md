# ICV Gateway

Enables an Emtron motorsports ECU to control the factory idle control
valve (ICV) actuators from a BMW S65/S85 V8/V10 engine.

A customer swapped a more powerful engine from a newer BMW M-series into their older E36 M3. On those engines, the factory ECU controls the ICVs over CAN using a proprietary and undocumented message format.

We reverse engineered this format. Our setup was simply an ICV on a bench supply and a laptop with a PCAN-USB interface. Using a CAN trace of the factory ECU talking to the ICVs, plus trial-and-error and deduction, we found the minimal messages needed to wake and control the ICV. The hardest parts to figure out were an init routine on power-up, a rolling counter, a checksum, and a mode byte.

The next challenge was that the Emtron ECU has very limited CAN capability. It can only break a CAN message into four 16-bit words, then send or read each one with a scale and offset applied. So we needed something that could sit across two CAN busses and translate the ECU's commands into the format the ICV needs.

It also had to be robust enough for the environment on a race car. An MRS Micro Gateway met our requirements. MRS provided a basic GUI tool that could create a blank project with their C library, then build and flash it. The documentation was missing some details, but we figured them out with some finagling.

I wrote all of the code in this repo from scratch.

## Files

- `main_hooks.c`: the functions the MRS runtime calls at boot, on every loop, and when a CAN message is received
- `icv_gateway.h`: bus numbers, CAN IDs, message constants, timeouts, and build flags
- `icv_gateway.c`: everything else

As committed, only `DUAL` is enabled, so this is a two-ICV build that follows the ECU.

## How it works

Every 5 ms, `usercode()` sends one command to the ICVs, sends the ICV positions back to the ECU, checks for timeouts, and then enters or leaves failsafe based on the error flags.

`SendICV`, `ReceiveICV`, `ReceiveECU` and `CheckTimeout` are function pointers. They get set at boot and pointed at different functions as the module changes state. When `init_ICV()` finishes the power-up handshake, it points `SendICV` at `control_ICV` and `CheckTimeout` at `check_op_timeout`. After that the init code never runs again and the main loop doesn't have to check a mode flag.

Timeouts are counted in loop ticks. For example, `ECU_TIMEOUT_VAL` is 400 ticks, which is 2 seconds at 5 ms per tick.

`control_ICV()` builds the command frame. It splits the 10-bit position (0 to 1000) across two bytes and sets an enable bit. In dual builds the position is copied for the second ICV. A rolling counter in the high nibble of byte 6 goes up on every frame. The checksum (`calc7bchk()`, an XOR of bytes 0 to 6) is calculated last, after everything else in the frame is set.

`store_ICV_pos()` pulls the commanded and actual positions out of the ICV status frame. They're packed across nibble boundaries, and I worked out the bit shifting by comparing known commands with the captured responses.

`relay_ECU()` sends the ICV positions back to the ECU on `0x640`. The gateway only translates. The ECU does the actual idle control.

## Build flags

These are set in `icv_gateway.h`. They're checked with `#ifdef`, so to turn one off you have to comment it out. Setting it to `FALSE` still turns it on.

- `DUAL`: two ICVs (S85 V10) instead of one (S65 V8). This changes the mode byte, the handshake, and the message sent back to the ECU.
- `INIT`: run the power-up handshake. Skip it if the ICVs are already awake.
- `ROBOTEST`: run random position sweeps instead of following the ECU, for bench testing.
- `DEBUG`: send a CAN frame at each decision point, tagged with the line number.

## Errors and failsafe

The gateway keeps two error bit fields and sends them on `0x641`. The bits are listed in the comment at the top of `icv_gateway.c`. Receive errors cover wake and init timeouts, ECU and ICV message timeouts, out-of-range positions, and a system fault reported by the ECU. Transmit errors cover failsafe being active and failed sends on either bus.

Any receive error other than wake or init is treated as critical. When one shows up, `manage_failsafe()` sets the position to 30% (`0x12C` out of 1000) and sets the failsafe flag, and `receive_ECU()` stops taking requests from the ECU. Once all the critical errors clear, it goes back to normal.

## Debugging

There was no way to attach a debugger, so with `DEBUG` defined, `send_debug_packet(__LINE__, ...)` sends a CAN frame tagged with the line number it came from. That let me follow the code on a CAN analyzer while it was running, without stopping it and throwing off the timing.

## Bench testing

With `ROBOTEST` defined, `robo_test()` and `debug_cycle_position()` sweep the ICVs from 0 to 1000 and back at random speeds, with random holds and jumps. This ran on the bench for at least two days straight without a fault. There was never a long bench test with the ECU driving it, but it ran in the customer's car without problems for the rest of my time at the shop.

## Known issue

`check_cmd_position()` checks `position < 0` on a `uint16_t`, which can never be true. Only the upper limit of 1000 is actually checked.
