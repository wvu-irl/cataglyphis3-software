#!/bin/bash
taskset -c 0,1 roslaunch hw_interface NB3_UDP_Receiver.launch
