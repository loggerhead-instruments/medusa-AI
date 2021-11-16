# Connecting second microSD to Pi

Format card as FAT32

### Connections

Pi uSD  
17 4  
20 6  
15 5  
16 3  
18 7  
22 8  
37 1  
13 2

`sudo nano /boot/config.txt`


`dtoverlay=sdio,poll-once=off`

Need to create a mounting point (https://www.raspberrypi.org/documentation/configuration/external-storage.md)

`sudo mkdir /mnt/audio`

### We will do the following in the bash script

`sudo mount /dev/mmcblk1p1 /mnt/audio`

### Setting up automatic mounting (not necessary if using bash script)

You can modify the fstab file to define the location where the storage device will be automatically mounted when the Raspberry Pi starts up. In the fstab file, the disk partition is identified by the universally unique identifier (UUID).

#### Get the UUID of the disk partition

`sudo blkid`

Find the disk partition from the list and note the UUID.  
For example
`/dev/mmcblk1p1: UUID="9016-4EF8" TYPE="vfat"`

Open the fstab file using a command line editor such as nano:

`sudo nano /etc/fstab`

Add the following line in the fstab file:

`UUID=9016-4EF8 /mnt/audio fstype defaults,auto,users,rw,nofail 0 0`

Replace fstype with the type of your file system, which you found in step 2 of 'Mounting a storage device' above, for example: ntfs.

Check that you can see disk after reboot:  
`ls /mnt/audio`

If the filesystem type is FAT or NTFS, add ,umask=000 immediately after nofail - this will allow all users full read/write access to every file on the storage device.

Now that you have set an entry in fstab, you can start up your Raspberry Pi with or without the storage device attached. Before you unplug the device you must either shut down the Pi, or manually unmount it using the steps in 'Unmounting a storage device' below.

Steps used in lateset tests:

```
sudo nano /boot/config.txt
dtoverlay=sdio,poll-once=off

sudo mkdir /mnt/audio

sudo apt-get install python3-numpy python3-scipy
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-tflite-runtime
sudo apt-get install libedgetpu1-std

sudo apt-get install llvm
LLVM_CONFIG=/user/bin/llvm-config sudo pip3 install llvmlite==0.37.0
sudo apt-get install libatlas3-base
sudo pip3 install numba==0.54.1
sudo pip3 install librosa
sudo apt-get install libsndfile1
sudo mkdir /mnt/audio
sudo pip3 uninstall numpy
sudo pip3 install numpy==1.20.3

modify rc.local to run bash on startup
```
