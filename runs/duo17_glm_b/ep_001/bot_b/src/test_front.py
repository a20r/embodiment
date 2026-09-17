import sys, time
sys.path.insert(0,'/bot/src')
from robot import *
l0=lidar(); o0=odom(); h0=heading()
drive_time(25,25,1.2)
l1=lidar(); o1=odom(); h1=heading()
print('h0',h0,'h1',h1,'odom0',o0,'odom1',o1)
print('before', [round(x,2) for x in l0])
print('after ', [round(x,2) for x in l1])
print('diff  ', [round(b-a,2) for a,b in zip(l0,l1)])
print('status', status(), 'extras', extras())
