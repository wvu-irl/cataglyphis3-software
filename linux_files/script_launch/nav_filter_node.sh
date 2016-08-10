#!/bin/bash
taskset -c 0,1 rosrun navigation navigation_filter_node
