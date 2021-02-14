from time import sleep
import RPi.GPIO as GPIO
import sys, os, glob, time
import datetime
from subprocess import call
import wave

print("audioProcess")

statusPin = 24;
statusPin2 = 26;


GPIO.setmode(GPIO.BOARD)
GPIO.setup(statusPin, GPIO.OUT)
GPIO.setup(statusPin2, GPIO.OUT)
GPIO.output(statusPin, 1)
GPIO.output(statusPin2, 0)


# Process audio here
# - read files
# - process files
# - save detection counts

# simulate processing delay
print("Processing")
sleep(10)

file1 = open("/mnt/audio/detections.txt", "w")
file1.write("w:10\n")
file1.close()

print("File written")

GPIO.output(statusPin, 0)
GPIO.output(statusPin2, 1)

sleep(2)

# call("sudo nohup shutdown -h now", shell=True)
exit()

