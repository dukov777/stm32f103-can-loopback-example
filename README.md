# stm32f103-can-loopback-example
This code has been tested via SocketCan on Linux.
Once PCAN interface is attached check the if ***can0*** is shown in network devices.

`ip a`

then check the `can0` status

`ip -details -statistics link show can0`

Restart interface if needed:

`sudo ip link set can0 type can restart-ms 100`

Before do any actions on SocketCan ensure that CAN interface is set to appropriate bit rate

`sudo ip link set can0 up type can bitrate 125000`

Monitor CAN messages. Shows input and output

`candump -ta can0`

Use this to send EXTENDED messages

`cansend can0 11223344#11.22`

For STANDART messages

`cansend can0 123#11.22`


