from lib import *
h0=rf('d4'); l0=ri('d9'); r0=ri('d6')
w('d1',10); w('d7',-10); time.sleep(2); stop(); time.sleep(0.3)
h1=rf('d4'); l1=ri('d9'); r1=ri('d6')
print("spin l+r-: dh",h1-h0,"dl",l1-l0,"dr",r1-r0)
print("lidar",lidar())
# now forward calibration in open space: front beam?
