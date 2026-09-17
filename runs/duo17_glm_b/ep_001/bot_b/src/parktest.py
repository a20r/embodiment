import time
def last(v):
    parts=[x for x in v.split('\n') if x.strip()]
    return parts[-1] if parts else ''
LOG=open('/memory/parktest.log','a')
t0=time.time()
with open('/dev/robot/d1','w') as f: f.write('0\n')
with open('/dev/robot/d7','w') as f: f.write('0\n')
vals=[]
while time.time()-t0<170:
    try: v=float(last(open('/dev/robot/d11').read().strip()))
    except: v=-1
    if v>=0: vals.append((time.time()-t0,v))
    if int(time.time()-t0)%10==0:
        LOG.write('%.0f d11=%.3f\n'%(time.time()-t0,v)); LOG.flush()
    time.sleep(0.5)
o9=last(open('/dev/robot/d9').read()); o6=last(open('/dev/robot/d6').read())
LOG.write('PARKTEST done n=%d first=%.3f last=%.3f min=%.3f max=%.3f odom %s %s\n'%(len(vals),vals[0][1],vals[-1][1],min(v for _,v in vals),max(v for _,v in vals),o9,o6))
