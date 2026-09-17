import ctl, rb, time, math, json, os
import explore
from explore import L
ctl.CPU=1920.0
SIDE=int(os.environ.get('SIDE','R'))if os.environ.get('SIDE','R').isdigit() else os.environ.get('SIDE','R')
TARGET=0.17; SPEED=38
laststatus=['']; lastcomm=[0]; lastrec=[0]
def comm(b,d11):
    st=rb.rd('d3',0.3)
    core=st.split('tick=')[-1].split(' ',1)[-1].split(' tx')[0]
    if core!=laststatus[0]: L('*** STATUS CHANGE',st); laststatus[0]=core
    d0=rb.rd('d0',0.3)
    if d0.strip()!='0': L('*** d0 =',d0)
    if time.time()-lastcomm[0]>20:
        lastcomm[0]=time.time()
        rb.wr('d8',f'B: wall-following, at my ({b.x:.1f},{b.y:.1f}) = your ({b.x+7.9:.1f},{b.y:.1f}). status {core}. d11={d11}')
    for _ in range(8):
        rx=rb.rd('d10',0.3)
        if not rx.strip(): break
        open('/bot/src/msgs.txt','a').write(f'{time.time():.0f} RX: {rx}\n')
        if 'GOAL' in rx.upper(): L('!!! RX GOAL:',rx[:200])
        elif 'homing' not in rx: L('RX:',rx[:160])
def main():
    b=ctl.Bot()
    last=json.loads(open('/bot/src/pose.jsonl').readlines()[-1]); b.x,b.y=last['x'],last['y']
    right = (SIDE=='R')
    # beam indices: front 0; right side = 12 (h+270), front-right 14, left 4, front-left 2
    S = 12 if right else 4; FS = 14 if right else 2; FS2 = 15 if right else 1
    L(f'WALL start {b.x:.2f},{b.y:.2f} h {b.h:.0f} side {SIDE}')
    t0=time.time(); n=0; mode='follow'; tm=time.time()
    while not os.path.exists('/bot/src/STOP'):
        b.update(); s=b.scan()
        if not s: continue
        s=[v if v>0 else 3.0 for v in s]
        front=min(s[0],s[1] if s[1]<0.3 else 3, s[15] if s[15]<0.3 else 3)
        side=s[S]; fside=s[FS]; fside2=s[FS2]
        bump=b.bump()
        n+=1
        if n%3==0:
            st=rb.rd('d3',0.3)
            if 'here=1' in st: L('!!!!! HERE=1 at',round(b.x,2),round(b.y,2)); break
        if n%10==0:
            d11=rb.rd('d11',0.3); comm(b,d11)
            try:
                if float(d11)>0.985: L('d11>0.9 stop at',round(b.x,2),round(b.y,2)); break
            except: pass
            if time.time()-lastrec[0]>3: lastrec[0]=time.time(); b.record('wall'); 
            if n%50==0: L(f'wall {b.x:.2f},{b.y:.2f} h{b.h:.0f} d11 {d11} front {front:.2f} side {side:.2f} mode {mode}')
        if bump:
            b.set(-30,-30); time.sleep(0.5); b.stop()
            mode='turnaway'; tm=time.time(); L('bump; turning away'); continue
        if mode=='turnaway':
            # rotate away from wall side until front clear
            if right: b.set(25,-25)
            else: b.set(-25,25)
            if front>0.35 and time.time()-tm>0.3: mode='follow'
            if time.time()-tm>4: mode='follow'
            time.sleep(0.05); continue
        if front<0.2:
            mode='turnaway'; tm=time.time(); continue
        # steering: positive corr = turn toward increasing heading (left)
        if side>0.45 and fside>0.4:
            # opening on wall side: turn toward it
            corr = -22 if right else 22
            sp=SPEED-10
        else:
            err = side-TARGET           # >0: too far from wall -> turn toward wall
            k=60*err
            k=max(-18,min(18,k))
            corr = -k if right else k
            # front-side proximity: turn away
            if fside<0.16: corr += 12 if right else -12
            sp=SPEED if front>0.4 else 26
        b.set(sp+corr, sp-corr)
        time.sleep(0.04)
    b.stop(); L('WALL stop')
if __name__=='__main__':
    try: main()
    except Exception:
        import traceback; L('EXC',traceback.format_exc())
    finally: rb.wr('d1','0'); rb.wr('d7','0')
