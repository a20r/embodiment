import time,statistics,sys
sys.path.insert(0,'/bot/src')
def last(v):
    parts=[x for x in v.split('\n') if x.strip()]
    return parts[-1] if parts else ''
def rd(p):
    try: return last(open(f'/dev/robot/{p}').read().strip())
    except Exception: return ''
LOG=open('/memory/d11test.log','a')
def L(s): LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
def phase(name,dur,txrate=0,drive=(0,0)):
    L('PHASE %s start'%name)
    t0=time.time(); nxt=0; vals=[]
    while time.time()-t0<dur:
        if drive!=(0,0):
            with open('/dev/robot/d1','w') as f: f.write(str(drive[0])+'\n')
            with open('/dev/robot/d7','w') as f: f.write(str(drive[1])+'\n')
        if txrate and time.time()>nxt:
            with open('/dev/robot/d8','w') as f: f.write('B-TO-A: test\n')
            nxt=time.time()+1.0/txrate
        try: v=float(rd('d11'))
        except: v=-1
        if v>=0: vals.append((time.time()-t0,v))
        time.sleep(0.2)
    with open('/dev/robot/d1','w') as f: f.write('0\n')
    with open('/dev/robot/d7','w') as f: f.write('0\n')
    if len(vals)>4:
        a=statistics.mean(v for t,v in vals[:10]); b=statistics.mean(v for t,v in vals[-10:])
        L('PHASE %s end first=%.3f last=%.3f slope=%+.5f/s min=%.3f max=%.3f'%(name,a,b,(b-a)/(vals[-10][0]-vals[0][0]+1e-9),min(v for _,v in vals),max(v for _,v in vals)))
L('d11 3-phase test begin')
phase('STILL_NOTX',45)
phase('STILL_TX2Hz',45,txrate=2)
phase('DRIVE_NOTX',40,drive=(20,20))
L('d11 test done')
