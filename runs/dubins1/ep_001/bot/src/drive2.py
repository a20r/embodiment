import sys, time
v1, v3, dur = sys.argv[1], sys.argv[2], float(sys.argv[3])
def w(p,v):
    with open(f'/dev/robot/d{p}','w') as f: f.write(str(v)+'\n')
end=time.time()+dur
while time.time()<end:
    w(1,v1); w(3,v3); time.sleep(0.05)
w(3,0); w(1,0)
