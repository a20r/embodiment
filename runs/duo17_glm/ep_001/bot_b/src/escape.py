import os, select, time
LOG=open('/memory/escape.log','a',buffering=1)
def log(s): LOG.write(f"[{time.strftime('%H:%M:%S')}] {s}\n"); print(s,flush=True)
def rd(p,to=0.04):
    fd=os.open('/dev/robot/'+p,os.O_RDONLY|os.O_NONBLOCK)
    r,_,_=select.select([fd],[],[],to)
    v=None
    if r:
        try: v=os.read(fd,4096).decode().strip()
        except: v=None
    os.close(fd); return v
def wr(p,v):
    try:
        fd=os.open('/dev/robot/'+p,os.O_WRONLY)
        os.write(fd,(str(v)+'\n').encode()); os.close(fd)
    except Exception as e: log(f"WRERR{p}:{e}")
def profile():
    v=rd('d2')
    try: return [float(x) for x in v.split(',')]
    except: return None
def get(p):
    try: return float(rd(p))
    except: return None

log("escape start")
tryN=0
while tryN<8:
    tryN+=1
    # forward attempt 5s, monitor
    wr('d1','1'); wr('d7','1')
    e60,e90=get('d6'),get('d9'); t0=time.time(); moved=False; bump=False
    while time.time()-t0<5:
        time.sleep(1.0)
        e6,e9=get('d6'),get('d9'); d5=rd('d5'); d0=rd('d0')
        if d5=='1' or d0=='1': bump=True
        if e6 is not None and e60 is not None and (abs(e6-e60)+abs(e9-e90))>0: pass
    wr('d1','0'); wr('d7','0')
    e61,e91=get('d6'),get('d9')
    P=profile()
    log(f"try{tryN} fwd enc=({e60}->{e61},{e90}->{e91}) bump={bump} P={P}")
    if bump:
        # reverse 2.5s then rotate 45deg alternating
        wr('d1','-1'); wr('d7','-1'); time.sleep(2.5); wr('d1','0'); wr('d7','0'); time.sleep(0.5)
        if tryN%2==1: wr('d1','2'); wr('d7','-2')
        else: wr('d1','-2'); wr('d7','2')
        time.sleep(13)
        wr('d1','0'); wr('d7','0'); time.sleep(0.5)
        log(f"try{tryN} did escape: reverse+rotate, now h={rd('d4')} P={profile()}")
    else:
        # moved without bump: check if actually translating (profile drift)
        log(f"try{tryN} no bump; h={rd('d4')} e=({e61},{e91})")
        time.sleep(2)
log("escape end")
