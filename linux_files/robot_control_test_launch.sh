#!/bin/bash
source /opt/ros/indigo/setup.bash
source ~/cataglyphis_ws/devel/setup.bash

gnome-terminal --tab -e "bash cataglyphis_ws/src/linux_files/script_launch/roscore.sh" --tab -e "bash -c 'sleep 1 && bash cataglyphis_ws/src/linux_files/script_launch/voice_node.sh'" --tab -e "bash -c 'sleep 2 && bash cataglyphis_ws/src/linux_files/script_launch/simulation_node.sh'" --tab -e "bash -c 'sleep 3 && bash cataglyphis_ws/src/linux_files/script_launch/safe_pathing_node.sh'" --tab -e "bash -c 'sleep 4 && bash cataglyphis_ws/src/linux_files/script_launch/map_manager_node.sh'" --tab -e "bash -c 'sleep 5 && bash cataglyphis_ws/src/linux_files/script_launch/HSM_Master_Executive.sh'" --tab -e "bash -c 'sleep 6 && bash cataglyphis_ws/src/linux_files/script_launch/exec_node.sh'" --tab -e "bash -c 'sleep 7 && bash cataglyphis_ws/src/linux_files/script_launch/mission_planning_node.sh'"
