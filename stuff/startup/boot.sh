wget 192.168.0.11/fpga.bin -O /home/pi/startup/fpga.bin
dtoverlay spi0-hw-cs
/home/pi/icehat/ice_tool/ice_tool /home/pi/startup/fpga.bin
dtoverlay -r spi0-hw-cs
