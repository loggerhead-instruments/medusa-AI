#!/bin/bash
sudo mount /dev/mmcblk1p1 /mnt/audio
sudo /usr/bin/python3 /home/pi/medusa/audioProcess.py &
