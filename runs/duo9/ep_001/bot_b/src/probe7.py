import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time
# drive forward toward most open lidar direction? first: what direction is beam0?
# rotate until beam with max range is at index 0? Simpler: just measure forward motion vs lidar.
l0 = lidar()
print("lidar", l0, flush=True)
h = read_port("d1")
print("hdg", h, flush=True)
# forward slowly, watch beams
write_port("d10","10"); write_port("d11","10")
for i in range(6):
    time.sleep(1)
    print(i, "hdg",read_port("d1"), "e",read_port("d7"),read_port("d8"), "d5",read_port("d5"), "lidar",read_port("d3"), flush=True)
write_port("d10","0"); write_port("d11","0")
