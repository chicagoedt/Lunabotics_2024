quick fix for the jetson not updating ip address/pyscript finding an error:

sudo chmod 666 /dev/ttyACM0

**note this fix only applies on that session, restarting the jetson will need us to apply this fix once more.

^ since tested, and error no longer displays. ill keep the fix there just incase we need it last minute.

the ardiuno is written to be connected at ttyACM0, so make sure its connected to that! otherwise the script will NOT work. (ie, the first usb port on the top left.)


temp reading:
i wrote this file to read from /sys/devices/virtual/thermal/thermal_zone1/temp

if it isnt there, use
ls /sys/devices/virtual/thermal/
to list thermal zones;
use
cat /sys/devices/virtual/thermal/thermal_zoneX/temp
to list tempratures of each thermal zones.


time.sleep(0.1) was added at the end of the py script simply because i think it would use too much cpu, and it would be evident from the temp



