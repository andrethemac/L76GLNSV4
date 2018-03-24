from pytrack import Pytrack
from L76GNSV4 import L76GNSS
import gc

print('gc.freemem',gc.mem_free())
py = Pytrack()
timeout = 180  # 3 minutes
print('gc.freemem',gc.mem_free())
l76 = L76GNSS(py, timeout=timeout)

print('gc.freemem',gc.mem_free())
print("getFix",l76.getFix())
print("coordinates",l76.coordinates())
print("getLocation",l76.getLocation())
print("getUTCTime",l76.getUTCTime())
print("getUTCDateTime",l76.getUTCDateTime(timeout=5))
print("getSpeed",l76.getSpeed())
print("getSpeedRMC",l76.getSpeedRMC())
print("getMessage",l76.gpsMessage())
print("getMessage(RMC)",l76.gpsMessage(messagetype='RMC',timeout=10))

print('gc.freemem',gc.mem_free())
