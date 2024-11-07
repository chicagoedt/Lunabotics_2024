quick fix for the jetson not updating ip address/pyscript finding an error:
sudo chmod 666 /dev/ttyACM0

**note this fix only applies on that session, restarting the jetson will need us to apply this fix once more.

the ardiuno is written to be connected at ttyACM0, so make sure its connected to that! otherwise the script will NOT work. (ie, the first usb port on the top left.)


temp reading:
i wrote this file to read from /sys/devices/virtual/thermal/thermal_zone1/temp

now, it might be different, and it will be updated wherever it is, but with base install it should be there ( i hope)

if it isnt there, use
ls /sys/devices/virtual/thermal/
to list thermal zones;
use
cat /sys/devices/virtual/thermal/thermal_zoneX/temp
to list tempratures of each thermal zones.


time.sleep(0.1) was added at the end of the py script simply because i think it would use too much cpu, and it would be evident from the temp


the new updated files are marked with test after the original name.
ill test it and then update the files as they are probably needed to be edited lol
