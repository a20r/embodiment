from lib import *
import time
print("h",rf('d4'))
print("lidar0",lidar())
# turn left wheel back, right fwd -> should decrease heading (ccw?)
w('d1',-10); w('d7',10); time.sleep(1.2); stop(); time.sleep(0.3)
print("h",rf('d4'))
print("lidar1",lidar())
