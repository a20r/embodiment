import time, threading, statistics
from rd import rd, wr
stop=False
def pump(cmds,hz=50):
    while not stop:
        for p,v in cmds.items():
            try: wr(p,v)
            except Exception: pass
        time.sleep(1.0/hz)
def watch(tag,dur=2.5):
    t0=time.time(); n=0; sums=[0.0]*16; d3s=[]; d0s=[]; d8=''
    while time.time()-t0<dur:
        d5=rd('d5',0.05)
        if d5:
            try:
                vals=[float(x) for x in d5.split(',')]
                for i,v in enumerate(vals):
                    if v>=0: sums[i]+=v
            except: pass
            n+=1
        d3=rd('d3',0.02)
        try: d3s.append(float(d3))
        except: pass
        d0=rd('d0',0.02)
        try: d0s.append(float(d0.split(',')[2]))
        except: pass
        d8=rd('d8',0.02) or d8
        time.sleep(0.05)
    m=[s/max(n,1) for s in sums]
    asym=m[:8] and (sum(m[1:4])+m[7])-0
    print(f'{tag}: n={n} d3={statistics.mean(d3s) if d3s else "-"} d0z={statistics.mean(d0s) if d0s else "-"} d8={d8}')
    print('   beams:', ' '.join(f'{x:.2f}' for x in m))
watch('baseline',2.0)
for tag,cmds in [
    ('d7=0.5,d4=0', {'d7':'0.5','d4':'0'}),
    ('d7=1,d4=0.3', {'d7':'1','d4':'0.3'}),
    ('d7=-1,d4=-0.3',{'d7':'-1','d4':'-0.3'}),
    ('d7=1000,d4=30',{'d7':'1000','d4':'30'}),
]:
    stop=False
    th=threading.Thread(target=pump,args=(cmds,)); th.start()
    watch(tag,2.5)
    stop=True; th.join()
    watch(tag+' (after)',1.5)
