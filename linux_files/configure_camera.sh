#!/bin/bash

v4l2-ctl --device=/dev/video0 --set-ctrl focus_auto=0
v4l2-ctl --device=/dev/video0 --set-ctrl focus_absolute=5
v4l2-ctl --device=/dev/video0 --set-ctrl white_balance_temperature_auto=1
v4l2-ctl --device=/dev/video0 --set-ctrl exposure_auto_priority=0
#v4l2-ctl --device=/dev/video0 --set-ctrl white_balance_temperature=4000
