# Connecting second microSD to Pi

Format card as FAT32

### Connections
Pi   uSD  
17    4  
20    6  
15    5  
16    3  
18    7  
22    8  
37    1  
13    2  


`sudo nano /boot/config.txt`

add
`dtoverlay=sdio,poll-once=off`


Need to create a mounting point (https://www.raspberrypi.org/documentation/configuration/external-storage.md)

`sudo mkdir /mnt/audio`
`sudo mount /dev/mmcblk1p1 /mnt/audio`



### Setting up automatic mounting
You can modify the fstab file to define the location where the storage device will be automatically mounted when the Raspberry Pi starts up. In the fstab file, the disk partition is identified by the universally unique identifier (UUID).

### Get the UUID of the disk partition:

`sudo blkid`  

Find the disk partition from the list and note the UUID. For example, 5C24-1453.
Open the fstab file using a command line editor such as nano:

`sudo nano /etc/fstab`  

Add the following line in the fstab file:

`UUID=5C24-1453 /mnt/mydisk fstype defaults,auto,users,rw,nofail 0 0`


Replace fstype with the type of your file system, which you found in step 2 of 'Mounting a storage device' above, for example: ntfs.

If the filesystem type is FAT or NTFS, add ,umask=000 immediately after nofail - this will allow all users full read/write access to every file on the storage device.  

Now that you have set an entry in fstab, you can start up your Raspberry Pi with or without the storage device attached. Before you unplug the device you must either shut down the Pi, or manually unmount it using the steps in 'Unmounting a storage device' below.