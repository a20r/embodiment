import os, select, time
D='/dev/robot/'
def read(p, timeout=0.3):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,2000000).decode().strip()
        except: out=''
    os.close(fd); return out

for trial in range(3):
    s=read('d2')
    far=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>1.2: far.append((round(r,2),round(e,2),round(a,2)))
        except: pass
    print(f"trial {trial}: n_far={len(far)}")
    from collections import Counter
    ce=Counter(e for r,e,a in far)
    print("  elev dist of far returns:", sorted(ce.items())[:15])
    if far: print("  sample:", far[:10])
    time.sleep(0.3)
