from time import sleep
import RPi.GPIO as GPIO
import sys, os, glob, time
import datetime
from subprocess import call
import wave

import librosa as lr
import math
import numpy as np
import os
from pathlib import Path
import sounddevice as sd
import soundfile as sf
import tflite_runtime.interpreter as tflite
import time

cfg = {'inwav_dir': '/mnt/audio',
'outwav_dir':'/mnt/audio/outwavs',
'save_specs': True,
'MODEL_PATH': '/home/pi/medusa/model/mnv2-singlegpu-epoch200-128batchsmallspec_postquant_float32inout_edgetpu_16.tflite'}

samplerate = 44100
chunk_seconds = 1.0
chunk_length = int(samplerate*chunk_seconds)
minute_length = int(44100*60)
whistle_counter = 0

if __name__ == "__main__":

    print("audioProcess")

    statusPin = 24
    statusPin2 = 26


    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(statusPin, GPIO.OUT)
    GPIO.setup(statusPin2, GPIO.OUT)
    GPIO.output(statusPin, 1)
    GPIO.output(statusPin2, 0)

    start_time = time.time()
    # Load TFLite model and allocate tensors.
    interpreter = tflite.Interpreter(model_path=cfg['MODEL_PATH'], 
    experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
    interpreter.allocate_tensors()

    # Get input and output tensors.
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    with os.scandir(cfg['inwav_dir']) as directory:
        for file in directory:
            if file.is_file() and file.name.endswith('.wav'):
                print("Loading Audio File: " + file.name)
            
                # Check if file has content
                if os.path.getsize(file.path):
                    data, samplerate = sf.read(file, dtype='float32')

                    first_minute_path = Path(cfg['outwav_dir']+'/'+file.name[:-4]+'~minute'+'.wav')
                    with sf.SoundFile(first_minute_path, mode='x', samplerate=44100,
                    channels=1, subtype='PCM_16') as audio_file:
                        audio_file.write(data[:minute_length])
            
                    number_of_chunks = math.ceil(len(data) / float(chunk_length))
                    my_list = [data[i * chunk_length:(i + 1) * chunk_length] for i in range(int(number_of_chunks))]

                    # Break sound into 1 second chunks
                    for i, chunk in enumerate(my_list):

                        if len(chunk) == 44100:
                            spectrogram = lr.amplitude_to_db(np.abs(lr.stft(chunk, n_fft=256, hop_length=145)))
                            spectrogram = spectrogram[3:, :]

                            spectrogram = np.expand_dims(spectrogram, axis=(-1,0))

                            # Make prediction
                            interpreter.set_tensor(input_details[0]['index'], spectrogram)
                            interpreter.invoke()

                            tflite_results = interpreter.get_tensor(output_details[0]['index'])


                            if np.any(tflite_results > .5):
                                
                                print("DOLPHIN SPOTTED!!")
                                whistle_counter += 1

                                if cfg['save_specs']:
                                    audio_file_path = Path(cfg['outwav_dir']+'/'+file.name[:-4]+'~'+str(i)+'~'+str(tflite_results[0,0])+'.wav')

                                    with sf.SoundFile(audio_file_path, mode='x', samplerate=44100,
                                    channels=1, subtype='PCM_16') as audio_file:
                                        audio_file.write(chunk)
                    
                    Path(file.path).unlink()
                        
                else:
                    print("ERROR: FILE SIZE 0 BYTES. SKIPPING...")
    print("PREDICTIONS COMPLETE")
    print(time.time() - start_time)



    # simulate processing delay
    #print("Processing")
    #sleep(10)

    file1 = open("/mnt/audio/detections.txt", "w")
    file1.write(f"w:{whistle_counter}\n")
    file1.close()

    print("File written")

    GPIO.output(statusPin, 0)
    GPIO.output(statusPin2, 1)

    sleep(2)

    # call("sudo nohup shutdown -h now", shell=True)
    exit()

