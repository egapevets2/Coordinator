This code implements a Zigbee Coordinator for an ESP32C6.  it is for my G-scale train network.  This code's purpose is a serial bridge between it's usb serial port, and all the end-devices in the network.  it also has functions for network RF 
levels and stuff.
This code contains addresses for all the end-devices and routers, and text names for each.  A possible future change is to 
allow changing the library of addresses and names via the usb port, instead of having that hard-coded.
