import time
ports=['d0','d2','d3','d4','d5','d6','d9','d11']
for i in range(6):
    line=[]
    for p in ports:
        try:
            with open('/dev/robot/'+p) as f:
                v=f.read().strip()
        except Exception as e:
            v='ERR'
        line.append(p+'='+v)
    print(' | '.join(line))
    time.sleep(1)
