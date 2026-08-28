import sys, time
v1, v3, dur = sys.argv[1], sys.argv[2], float(sys.argv[3])
def w(a,b):
    with open('/dev/robot/d1','w') as f: f.write(str(a)+'\n')
    with open('/dev/robot/d3','w') as f: f.write(str(b)+'\n')
end = time.time()+dur
while time.time() < end:
    w(v1,v3); time.sleep(0.05)
w(0,0)
