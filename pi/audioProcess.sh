#!/bin/bash
sudo mount /dev/mmcblk2p1 /mnt/audio
sudo /usr/bin/python3 /home/mendel/medusa/audioProcess.py &
