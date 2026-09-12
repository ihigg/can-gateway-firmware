# CAN Bus Gateway Firmware

This repo has two embedded C projects I wrote while working at Cohesion Motorsports, a vehicle electronics and motorsports engineering shop. In both projects, two systems in a car needed to talk to each other over CAN but used different message formats, and neither side could be changed. In both cases the solution was a small gateway module that sits across two CAN busses and translates between them.

- [icv-gateway](icv-gateway/) lets an Emtron ECU control the factory idle control valves (ICVs) from a BMW S65/S85 engine. This is the bigger of the two. It includes the reverse engineered ICV protocol, timeout and error reporting, a failsafe, and a bench test mode.
- [wheel-column-gateway](wheel-column-gateway/) lets a steering column switch module from one vehicle work with the body electronics of a different vehicle.

Both run on MRS Electronic gateway modules, which use a Freescale HCS12X 16-bit MCU with multiple CAN channels. There is no operating system, just a single main loop with timer interrupts.

## What's included

This is only my code. The ICV gateway comes from the `improve-error-reporting` branch at `2d52a79` (May 2023) and the wheel/column gateway comes from `master` at `3e9cda6` (October 2020). Later commits by other people were left out.

To publish it I renamed the files and updated the includes and header guard to match, rewrote the header comments, and removed the empty vendor interrupt stubs. In the ICV project I moved the two `CANBUS_ID_*` defines in from the vendor config header. In the wheel project I removed about 700 lines of MRS example comments from `main_hooks.c`. No logic was changed. The `TODO` comments are mine and are left as I wrote them.

## Building

This won't build as-is. The code depends on the MRS BIOS/OS framework, which is their IP and isn't included here. I also left out compiled files, CAN databases, schematics, bus traces, and anything that identifies the customer.

These are the framework calls used in the code:

- `os_timestamp(&t, OS_1ms)` saves a millisecond timestamp
- `os_time_past(t, n, OS_1ms)` is true once `n` ms have passed since `t`
- `os_can_send_msg(bus, id, len, data)` queues a CAN frame on a bus
- `bios_can_msg_typ` is a received frame with `.id`, `.len` and `.data[8]`
- `usercode_init()` runs once at boot and `usercode()` runs on every pass of the main loop
- `user_can_message_receive(hw_id, msg)` is called from the main loop for each received frame

Bus numbers are set up in the MRS project tool, not in this code.

If you want to read through it, start with `main_hooks.c` in either project. That's where the MRS runtime calls into my code. The rest of the logic is in the gateway `.c` file.
