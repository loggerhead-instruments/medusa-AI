#!/bin/bash
sudo mount /dev/mmcblk1p1 /mnt/audio
/usr/bin/python3 /home/pi/audioProcess.py &
