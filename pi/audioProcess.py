import time
import os
from subprocess import call
import librosa as lr
import logging
from logging.handlers import RotatingFileHandler
import math
import numpy as np
import os
from pathlib import Path
from periphery import GPIO
import shutil
import soundfile as sf
import tflite_runtime.interpreter as tflite


def storage_check(mounted_directory, required_gigs=2):
    """
    Determines if the specified drive has enough storage.

    Parameters
    ----------
    mounted_directory: str or Path
        The directory to analyze for free space
    required_gigs: int, optional
        The amount of gigabytes required on the drive (default is 2)
    
    Returns
    -------
    bool
        A flag that determined if the directory has enough storage.
    """
    bytes_in_gig = 1073741824
    total, used, free = shutil.disk_usage(mounted_directory)

    free_gigs = free//bytes_in_gig

    if free_gigs >= required_gigs:
        return True
    else:
        return False


cfg = {'inwav_dir': '/mnt/audio',
#'inwav_dir': '/home/mendel/medusa/1hour',
#'inwav_dir': '/media/mendel/18F7-2660',
'outwav_dir':'/mnt/audio/outwavs',
#'outwav_dir':'/media/mendel/18F7-2660/outwavs',
#'outwav_dir':'/home/mendel/medusa/outwavs',
'log_file': '/home/mendel/medusa/medusa.log',
'save_specs': True,
'MODEL_PATH': '/home/mendel/medusa/model/mnv2-singlegpu-epoch200-128batchsmallspec_postquant_float32inout_edgetpu_16.tflite'}

gpio_pin = 39
samplerate = 44100
chunk_seconds = 1.0
chunk_length = int(samplerate*chunk_seconds)
minute_length = int(44100*60)
whistle_counter = 0

if __name__ == "__main__":
    logging.Formatter.converter = time.gmtime
    logging.basicConfig(handlers=[RotatingFileHandler(filename=cfg['log_file'], maxBytes=524288, backupCount=2)], level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    try:
        print("audioProcess")

        status_pin = GPIO("/dev/gpiochip0", gpio_pin, "out")
        status_pin.write(True)

        Path(cfg['outwav_dir']).mkdir(exist_ok=True)

        start_time = time.time()
        # Load TFLite model and allocate tensors.
        interpreter = tflite.Interpreter(model_path=cfg['MODEL_PATH'], 
        experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
        interpreter.allocate_tensors()

        # Get input and output tensors.
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        num_wavs = len([wav for wav in os.listdir(cfg['inwav_dir']) if wav.endswith('.wav')])

        if num_wavs == 0:
            logging.info("No files to process.")
            
            # Teensy crash or debugging
            time.sleep(180)
            
            status_pin.write(False)
            status_pin.close()
            
            call("sudo nohup shutdown -h now", shell=True)
        else:
            logging.info(f"Processing {num_wavs} files...")
            
            enough_space = storage_check(cfg['inwav_dir'])
            
            with os.scandir(cfg['inwav_dir']) as directory:
                for file in directory:
                    if file.is_file() and file.name.endswith('.wav'):
                        print("Loading Audio File: " + file.name)
                    
                        # Check if file has content
                        if os.path.getsize(file.path):
                            data, samplerate = sf.read(file, dtype='float32')
                            
                            if enough_space:
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

                                    spectrogram = np.expand_dims(spectrogram, axis=-1)
                                    spectrogram = np.expand_dims(spectrogram, axis=0)

                                    # Make prediction
                                    interpreter.set_tensor(input_details[0]['index'], spectrogram)
                                    interpreter.invoke()

                                    tflite_results = interpreter.get_tensor(output_details[0]['index'])


                                    if np.any(tflite_results > .5):
                                        
                                        print("DOLPHIN SPOTTED!!")
                                        whistle_counter += 1

                                        if cfg['save_specs']:
                                            if enough_space:
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
            #time.sleep(10)

            file1 = open(f"{cfg['inwav_dir']}/detections.txt", "w")
            file1.write(f"w:{whistle_counter}\n")
            file1.close()

            print("File written")

            status_pin.write(False)
            status_pin.close()

            call("sudo nohup shutdown -h now", shell=True)
    except:
        logging.exception('Something went wrong...')
        
        status_pin.write(False)
        status_pin.close()

        call("sudo nohup shutdown -h now", shell=True)
