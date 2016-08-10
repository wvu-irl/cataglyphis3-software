#!/bin/bash
taskset -c 0,1 roslaunch hw_interface NB1_UDP_Sender.launch
