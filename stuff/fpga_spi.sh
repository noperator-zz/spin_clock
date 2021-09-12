#!/bin/bash

#    A script to configure Lattice iCE40 FPGA by SPI from Raspberry Pi
#
#    Copyright (C) 2015 Jan Marjanovic <jan@marjanovic.pro>
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.


echo ""
if [ $# -ne 1 ]; then
    echo "Usage: $0 FPGA-bin-file "
    exit 1
fi

if [ $EUID -ne 0 ]; then
    echo "This script must be run as root" 1>&2
    exit 1
fi


if [ ! -d /sys/class/gpio/gpio25 ]; then
    echo "GPIO 25 not exported, trying to export..."
    echo 25 > /sys/class/gpio/export
    if [ ! -d /sys/class/gpio/gpio25 ]; then
	echo "ERROR: directory /sys/class/gpio/gpio25 does not exist"
	exit 1
    fi
else
    echo "OK: GPIO 25 exported"
fi

echo ""
if [ -e /dev/spidev0.0 ]; then
    echo "OK: SPI driver loaded"
else
    echo "spidev does not exist"
    
    lsmod | grep spi_bcm2708 >& /dev/null

    if [ $? -ne 0 ]; then
	echo "SPI driver not loaded, try to load it..."
	modprobe spi_bcm2708

	if [ $? -eq 0 ]; then
	    echo "OK: SPI driver loaded"
	else
	    echo "Could not load SPI driver"
	    exit 1
	fi  
    fi
fi

echo ""
echo "Changing direction to out"
echo out > /sys/class/gpio/gpio25/direction
cat /sys/class/gpio/gpio25/direction

echo "Setting output to low"
echo 0 > /sys/class/gpio/gpio25/value
cat /sys/class/gpio/gpio25/value

echo ""
echo "Please power cycle the iCE40 FPGA board"
echo "Press any key..."
read

echo "Continuing with configuration procedure"
dd if=$1 of=/dev/spidev0.0

echo -e "\x0\x0\x0\x0\x0\x0\x0" > /dev/spidev0.0

echo "Setting output to high"
echo 1 > /sys/class/gpio/gpio25/value
