# L76GLNSV4
MicroPython library for quectel L76 glnss gps on pycom pytrack

2019-06-08
* added new features
* enterStandBy: put gps off, needs powercycle to restart
* hotStart: restart the GPS with last found data in non volitale ram
* warmStart: restart the GPS with last known data not in ram
* coldStart: as powercycle
* fullColdStart: forget everything and start al over, takes long to get fix (same as powercycle)
* setPeriodicMode: Mode 2 preserves the most energy
* setAlwaysOn: same as setPeriodicMode(0), keeps the gps awake


improved fix speed

improved nmea message handling


**example using the pycom pytrack and wipy3 or lopy4**

this example uses the machine.deepsleep to put the wipy/lopy to sleep, but the pytrack board keeps powered.
The L76 gps doesn't looses its rtc clock or the satelites in view, resulting in a very quick fix.
powering down the pytrack means you need to look anew for satelites, this takes a long time.
the Periodic StandBy mode seems to be the most enery efficient. (more test needs to be done)
Don't use the pytrack sleep, if you want quick fixes. The pytrack sleep powers the whole board down.


```python
from pytrack import Pytrack
from L76GNSV4 import L76GNSS
import machine, time
import pycom

print("up")
py = Pytrack()
L76 = L76GNSS(pytrack=py)
L76.setAlwaysOn()

print("print gsv")
# returns the info about sattelites in view at this moment
# even without the gps being fixed
print(L76.gps_message('GSV',debug=True))

print("gga")
# returns the number of sattelites in view at this moment
# even without the gps being fixed
print(L76.gps_message('GGA',debug=True)['NumberOfSV'])

L76.get_fix(debug=False)
pycom.heartbeat(0)
if L76.fixed():
    pycom.rgbled(0x000f00)
else:
    pycom.rgbled(0x0f0000)
print("coordinates")
# returns the coordinates
# with debug true you see the messages parsed by the
# library until you get a the gps is fixed
print(L76.coordinates(debug=False))
L76.getUTCDateTime(debug=True)

print("put gps in low power for 40 seconds after 20 seconds")
L76.setPeriodicMode(2,20000,40000,60000,60000)
print("wait 15 seconds")
for x in range(15,0,-1):
    pycom.rgbled(0x000f00+x)
    time.sleep(1)
print("put lopy to deepsleep for 1 minute ")
print("the gps schould be awake before the lopy")
machine.deepsleep(60000)
```
