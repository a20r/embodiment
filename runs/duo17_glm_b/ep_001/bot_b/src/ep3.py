import sys,time,math,statistics,random
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/ep3.log','a')
SCAN=open('/memory/scan_ep3.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass

random.seed(7)
ESC={'until':0.0}
pose={'x':0.0,'y':0.0,'dist':0.0}
o0=None; hprev=None
def upd():
    global o0,hprev
    o1=R.odom()
    if o0 is None: o0=o1; return
    dl=o1[0]-o0[0]; dr=o1[1]-o0[1]
    if abs(dl)>500 or abs(dr)>500: o0=o1; return
    o0=o1
    d=(dl+dr)/2*0.00066
    pose['dist']+=abs(d)
    h=R.heading()
    if hprev is not None:
        dh=((h-hprev+180)%360)-180
        if abs(dh)>90: return
        pose['x']+=d*math.cos(math.radians(h))
        pose['y']+=d*math.sin(math.radians(h))
    hprev=h

GRID=set()
def med_lidar(n=2,tries=20):
    scans=[]
    for _ in range(tries):
        s=R.lidar()
        if len(s)==16: scans.append(s)
        if len(scans)>=n: break
        time.sleep(0.02)
    if not scans: return None
    return [statistics.median([s[i] for s in scans]) for i in range(16)]

def txstate():
    st=R.status()
    for tok in st.split():
        if tok.startswith('tx='):
            return tok.split(':')[1] if ':' in tok else '?'
    return '?'

BN={'n':0}
def beacon(msg):
    BN['n']+=1
    try: R.tx('B-TO-A: %s #%d'%(msg,BN['n']))
    except Exception: pass

def sig(n=4,dt=0.05):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    return statistics.median(vs) if vs else 0

def choose_dir(l):
    def b(i):
        v=l[i%16]; return v if v>0 else 2.5
    right=min(b(10),b(11),b(12),b(13)); left=min(b(3),b(4),b(5),b(6))
    return (1 if right>left else -1),left,right

ESC_N={'n':0,'t':0.0}
def follow(dt=0.12):
    lwf=0.3
    omark=None
    seg=0
    while True:
        seg+=1
        l=med_lidar()
        if l is None: R.stop(); time.sleep(0.05); continue
        upd()
        GRID.add((int(pose['x']*2),int(pose['y']*2)))
        def b(i):
            v=l[i%16]; return v if v>0 else 2.5
        f=min(b(15),b(0),b(1))
        lw_use=b(4) if b(4)<2.4 else min(b(3),b(5))
        lwf=0.7*lwf+0.3*lw_use
        now=time.time()
        if now-ESC_N['t']>20: ESC_N['n']=0
        if f<=0.20:
            d,left,right=choose_dir(l)
            ESC_N['n']+=1; ESC_N['t']=now
            if ESC_N['n']>=3:
                L('UNJAM n=%d f=%.2f'%(ESC_N['n'],f))
                R.motors(-16,-16); time.sleep(1.3)
                R.motors(26,-26); time.sleep(2.8)
                R.motors(-13,-13); time.sleep(0.4)
                R.motors(20,20); time.sleep(2.0)
                R.stop(); ESC_N['n']=0
                continue
            L('ESC dir=%d f=%.2f L=%.2f R=%.2f n=%d'%(d,f,left,right,ESC_N['n']))
            R.motors(-13,-13); time.sleep(0.55)
            if d>0: R.motors(22,-11)
            else: R.motors(-11,22)
            time.sleep(1.25)
            R.stop()
            continue
        if f<=0.38:
            d,left,right=choose_dir(l)
            base=15
            if d>0: R.motors(base+10,base-8)
            else: R.motors(base-8,base+10)
            time.sleep(0.5)
            continue
        steer=max(-0.6,min(0.6,(lwf-0.30)*2.0))
        if b(3)<0.45 and b(5)>b(3)+0.2: steer=max(steer,0.3)
        if b(5)<0.45 and b(3)>b(5)+0.2: steer=min(steer,-0.3)
        base=20
        ml=int(base+steer*14); mr=int(base-steer*14)
        R.motors(max(-25,min(28,ml)),max(-25,min(28,mr)))
        time.sleep(dt)
        if seg%50==0:
            s=sig(2,0.04)
            L('FLW pos=%.1f,%.1f f=%.2f lw=%.2f d11=%.3f tx=%s cells=%d'%(pose['x'],pose['y'],f,lwf,s,txstate(),len(GRID)))
            SCAN.write('%.2f %.2f %s\n'%(pose['x'],pose['y'],','.join('%.2f'%v for v in l))); SCAN.flush()
        if omark is None: omark=(R.odom(),time.time())
        else:
            (o,t)=omark; o2=R.odom()
            if time.time()-t>12:
                if abs(o2[0]-o[0])+abs(o2[1]-o[1])<10:
                    L('NOMOVE - unjam')
                    ESC_N['n']=5; ESC_N['t']=0
                omark=(o2,time.time())

def main():
    L('EP3 start d11=%.3f tx=%s'%(sig(),txstate()))
    last_be=0; last_sum=0
    while True:
        try:
            st=R.status_d()
            if st.get('here')=='1':
                L('*** HERE=1 %s'%st); R.stop()
                for i in range(10):
                    beacon('AT GOAL here=1. A COME NOW')
                    time.sleep(0.5)
                time.sleep(2); continue
            if st.get('goal','0') not in ('0',''):
                L('*** GOALFLAG %s'%st)
            m=R.rx()
            if m: L('RX!! %s'%m[:150])
            ts=txstate()
            if ts in ('ok','busy'):
                L('*** PEER RANGE tx=%s - stop+burst'%ts)
                R.stop()
                for i in range(6):
                    beacon('HEAR YOU. STOPPED. SEND POS/HDG OR APPROACH ON MY d11 PEAK')
                    m=R.rx()
                    if m: L('RX!! %s'%m[:150])
                    time.sleep(0.6)
            if time.time()-last_be>2.4:
                last_be=time.time()
                beacon('B alive pos=%.1f,%.1f d11=%.2f seek A+GOAL'%(pose['x'],pose['y'],sig(2,0.03)))
            if time.time()-last_sum>45:
                last_sum=time.time()
                upd()
                L('SUM pos=%.1f,%.1f dist=%.1f cells=%d d11=%.3f tx=%s'%(pose['x'],pose['y'],pose['dist'],len(GRID),sig(3,0.04),txstate()))
            follow()
        except Exception as e:
            try: R.stop(); L('EXC %r'%e)
            except Exception: pass
            time.sleep(0.4)

if __name__=='__main__':
    main()
