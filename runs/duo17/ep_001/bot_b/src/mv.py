import rb, time, sys
def enc():
    a=rb.rd('d6',0.3); b=rb.rd('d9',0.3)
    try: return int(a), int(b)
    except: return None
def stop():
    for _ in range(2):
        rb.wr('d1','0'); rb.wr('d7','0')
def drive(vA, vB, dur, watch=True):
    """d1=vA (enc d9), d7=vB (enc d6). run for dur seconds, then stop"""
    e0=enc()
    rb.wr('d1',str(vA)); rb.wr('d7',str(vB))
    t0=time.time()
    while time.time()-t0<dur:
        time.sleep(0.05)
        if watch and rb.rd('d5',0.2)=='1' and (vA>0 or vB>0):
            pass
    stop()
    time.sleep(0.2)
    e1=enc()
    return e0,e1
def state():
    return dict(h=rb.rd('d4'), bump=rb.rd('d5'), d11=rb.rd('d11'), d0=rb.rd('d0'), d2=rb.rd('d2'))
if __name__=='__main__':
    vA=float(sys.argv[1]); vB=float(sys.argv[2]); dur=float(sys.argv[3])
    print('before', state())
    print('enc', drive(vA,vB,dur))
    print('after', state())
