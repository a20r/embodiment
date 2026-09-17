import time, threading
from rd import rd, wr
stop=False
def pump(cmds,hz=50):
    while not stop:
        for p,v in cmds.items():
            try: wr(p,v)
            except Exception: pass
        time.sleep(1.0/hz)
def run(tag,cmds,dur):
    global stop
    print(f'== {tag} ==', flush=True)
    stop=False
    th=threading.Thread(target=pump,args=(cmds,)); th.start()
    t0=time.time()
    while time.time()-t0<dur:
        print(f'  {time.time()-t0:4.1f} d0={rd("d0",0.03)} d1={rd("d1",0.03)} d2={rd("d2",0.03)} d3={rd("d3",0.03)} d6={rd("d6",0.03)}', flush=True)
        print(f'      d5={rd("d5",0.03)}', flush=True)
        time.sleep(0.2)
    stop=True; th.join()
run('stop (0,0)', {'d7':'0','d4':'0'}, 1.5)
run('d7=100', {'d7':'100','d4':'0'}, 3.0)
run('stop', {'d7':'0','d4':'0'}, 1.5)
run('d7=300', {'d7':'300','d4':'0'}, 3.0)
run('stop', {'d7':'0','d4':'0'}, 1.5)
