import time
buf=[]
last=0
with open('/dev/robot/d5') as f:
    while True:
        line=f.readline().strip()
        now=time.time()
        if line:
            try: buf.append(float(line))
            except: pass
        if now-last>2 and buf:
            last=now
            m=sum(buf)/len(buf)
            with open('/tmp/d5.log','a') as g:
                g.write('%.0f %.3f %d\n'%(now,m,len(buf)))
            buf=[]
