import time

def rd(p, retries=30, delay=0.05):
    for i in range(retries):
        try:
            with open('/dev/robot/'+p) as f:
                v=f.read().strip()
            if v!='':
                return v
        except Exception as e:
            v='ERR:'+str(e)
        time.sleep(delay)
    return v if isinstance(v,str) else 'ERR'

def wr(p, v):
    try:
        with open('/dev/robot/'+p,'w') as f:
            f.write(str(v)+'\n')
        return True
    except Exception as e:
        return False

def motors(l, r):
    wr('d1', l); wr('d7', r)

def stop():
    motors(0,0)

def heading():
    return float(rd('d4'))

def odom():
    return float(rd('d9')), float(rd('d6'))

def lidar():
    return [float(x) for x in rd('d2').split(',')]

def status():
    return rd('d3')

def extras():
    return rd('d0'), rd('d5'), rd('d11')

def drive_time(l, r, secs):
    motors(l,r); time.sleep(secs); stop()

def turn_to(target, tol=3.0, timeout=12):
    # turn in place toward absolute heading target (deg), shortest way
    t0=time.time()
    while time.time()-t0 < timeout:
        h = heading()
        err = ((target - h + 180) % 360) - 180
        if abs(err) <= tol:
            stop(); return True
        spd = max(-40, min(40, err*1.5))
        # left forward + right back turns CCW? test showed d1=30,d7=-30 -> heading +112 (CW increase)
        motors(spd if err>0 else -spd, -spd if err>0 else spd)
        time.sleep(0.15)
    stop(); return False

def drive_until(cb, l=30, r=30, timeout=10):
    t0=time.time()
    motors(l,r)
    while time.time()-t0 < timeout:
        if cb(): stop(); return True
        time.sleep(0.1)
    stop(); return False

def status_dict():
    s = rd('d3')
    out={}
    for tok in s.split():
        if '=' in tok:
            k,v=tok.split('=',1); out[k]=v
    return out

if __name__=='__main__':
    print('h',heading(),'odom',odom(),'lidar',lidar())
    print('status',status())
    print('extras',extras())
