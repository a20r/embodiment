import os, time, select, statistics as st

READ=['d0','d2','d4','d5','d6','d9','d10','d11']
def sample(dur):
    fds={n:os.open('/dev/robot/'+n,os.O_RDONLY|os.O_NONBLOCK) for n in READ}
    data={n:[] for n in READ}
    buf=''
    t0=time.time()
    while time.time()-t0<dur:
        r,_,_=select.select(list(fds.values()),[],[],0.02)
        for fd in r:
            for n,f in fds.items():
                if f==fd:
                    try:
                        raw=os.read(f,4096).decode()
                    except Exception: continue
                    if n=='d2':
                        buf+=raw
                        parts=buf.split(';')
                        buf=parts[-1]
                        data[n].extend(parts[:-1])
                    else:
                        d=raw.strip()
                        if d: data[n].append(d)
    for f in fds.values(): os.close(f)
    return data

def summ(tag,d):
    h=[float(x) for x in d['d4']]
    b=[float(x) for x in d['d11']]
    d2=[float(t.split(',')[0]) for t in d['d2'] if ',' in t]
    print(f"{tag}: d4={st.mean(h):.1f}±{st.pstdev(h):.1f} d11={st.mean(b):.3f} d2first~{st.mean(d2):.3f} d0={sorted(set(d['d0'][:60]))} d5={sorted(set(d['d5'][:20]))} d6={sorted(set(d['d6'][:20]))} d9={sorted(set(d['d9'][:20]))} radio={d['d10'][:2]}", flush=True)

def w(port,val):
    fd=os.open('/dev/robot/'+port,os.O_WRONLY|os.O_NONBLOCK)
    os.write(fd,(str(val)+"\n").encode()); os.close(fd)

summ("base", sample(2))
w('d1',0.5); summ("d1=0.5", sample(2.5))
w('d1',-0.5); summ("d1=-0.5", sample(2.5))
w('d1',0); summ("d1=0", sample(1.5))
w('d7',0.5); summ("d7=0.5", sample(2.5))
w('d7',-0.5); summ("d7=-0.5", sample(2.5))
w('d7',0); summ("d7=0", sample(1.5))
w('d1',100); summ("d1=100", sample(2.5))
w('d1',0); summ("d1=0b", sample(1))
w('d7',100); summ("d7=100", sample(2.5))
w('d7',0); summ("d7=0b", sample(1))
