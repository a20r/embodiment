import sys, time
port, val, dur = sys.argv[1], sys.argv[2], float(sys.argv[3])
end=time.time()+dur
while time.time()<end:
    with open(f'/dev/robot/d{port}','w') as f: f.write(val+'\n')
    time.sleep(0.05)
with open(f'/dev/robot/d{port}','w') as f: f.write('0\n')
