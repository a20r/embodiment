import sys,time,math,json
sys.path.insert(0,'/bot/src')
import explore2 as E
from explore2 import O,go,turn,front,log,snap_axis,d11,save_pose,c,m
RULE=-1  # -1 = left-hand (prefer heading-decrease side), +1 = right-hand
def ping():
    v=d11() or -1
    E.radio('A: GOAL MOVED (sync failed: arrivals must be <1min apart). Old spot is dead. I am exploring for new here=1. Rule: finder steps OUT, waits beside it, guides other; then both step IN together. d11=%.2f'%v)
def check():
    stt=m.rd('d3') or ''; rx=m.rd('d10'); d0=m.rd('d0'); d5=m.rd('d5')
    if rx:
        log('RX <<< %r'%rx); open('/bot/src/RX.txt','a').write(rx+'\n')
        if 'GOAL' in rx.upper() and 'no goal' not in rx.lower(): open('/bot/src/ALERT','a').write(rx+'\n'); log('!!! GOAL MSG'); c.stop(); return True
    if 'here=1' in stt:
        log('!!! FOUND here=1 %s at (%.2f,%.2f) h=%.0f L=%s'%(stt,O.x,O.y,O.h,O.L)); open('/bot/src/ALERT','a').write('FOUND %s at (%.2f,%.2f) h=%.0f\n'%(stt,O.x,O.y,O.h))
        open('/memory/NOTES.md','a').write('- ep1 %s NEW GOAL FOUND by explore3 at odom (%.2f,%.2f) h=%.0f lidar=%s\n'%(time.strftime('%H:%M'),O.x,O.y,O.h,O.L))
        c.stop(); c.drive(-45,-45); time.sleep(1.5); c.stop(); log('backed out: %s'%(m.rd('d3'))); return True
    if 'goal=1' in stt: log('!!! GOAL=1 %s'%stt); c.stop(); return True
    if d0 not in ('0',None): log('d0=%s at (%.2f,%.2f)'%(d0,O.x,O.y))
    return False
visits={}
def main():
    global RULE
    axis=snap_axis(O.h); turn(axis)
    while True:
        O.upd(); save_pose()
        if not O.L: continue
        L=O.L; F=front()
        left=(axis-90)%360; right=(axis+90)%360
        openL=L[12]>0.65; openR=L[4]>0.65; openS=F>0.5
        key=(round(O.x/0.5),round(O.y/0.5)); visits[key]=visits.get(key,0)+1
        if visits[key]%8==0: RULE=-RULE; log('revisits at %s -> rule %d'%(key,RULE))
        log('AT (%.2f,%.2f) h=%.0f axis=%.0f F=%.2f L=%.2f R=%.2f d11=%.2f open L%d S%d R%d'%(O.x,O.y,O.h,axis,F,L[12],L[4],d11() or -1,openL,openS,openR))
        pref=[(left,openL),(axis,openS),(right,openR)] if RULE==-1 else [(right,openR),(axis,openS),(left,openL)]
        chosen=None
        for ax,ok in pref:
            if ok: chosen=ax; break
        if chosen is None: chosen=(axis+180)%360
        if chosen!=axis: axis=chosen; turn(axis)
        if check(): return
        r=go(0.3,axis)
        if r=='wall': 
            # blocked immediately after turn; re-evaluate
            continue
        r=go(3.0,axis,stop_on_side=('L' if RULE==-1 else 'R'))
        log('  cruise -> %s at (%.2f,%.2f)'%(r,O.x,O.y))
        if r in ('openL','openR'): go(0.22,axis,min_front=0.2)
        ping()
        if check(): return
if __name__=='__main__':
    try: log('=== explore3 start rule %d'%RULE); main()
    finally: c.stop(); save_pose(); log('=== explore3 end')
