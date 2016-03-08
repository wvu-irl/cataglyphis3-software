#!/bin/sh
cd
cd ../../etc/udev/rules.d/

cat > 25-devices.rules << EOF
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux",ATTRS{idVendor}=="046d", ATTRS{idProduct}=="082d", ATTRS{serial}=="CAE8449F", SYMLINK+="video_camera"

KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A98B7DPT", SYMLINK+="servo_controller"

KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="FTXYDPJP", SYMLINK+="sun_sensor"

KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="FTX1OA40", SYMLINK+="sun_sensor"

KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="1027", ATTRS{idProduct}=="0003", ATTRS{iSerial}=="106", SYMLINK+="ranging_radio"

EOF
