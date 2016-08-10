#!/bin/bash
taskset -c 0,1 roslaunch hw_interface NB2_UDP_Receiver.launch
