#!/bin/bash
taskset -c 0,1 roslaunch hw_interface right_roboteq.launch
