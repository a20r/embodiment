import json, time, sys
def snap(n=6):
    time.sleep(0.3)
    lines=[json.loads(l) for l in open('/tmp/sensors.log')][-n:]
    import statistics as st
    scans=[[float(x) for x in l['d2'].split(',')] for l in lines if 'd2' in l]
    avg=[]
    for i in range(16):
        vals=[s[i] for s in scans if s[i]>=0]
        avg.append(st.median(vals) if vals else -1)
    hd=[float(l['d4']) for l in lines]
    # circular-ish median: just print median
    print(f"t={lines[-1]['t']} hd={st.median(hd):.1f} d6={lines[-1]['d6']} d9={lines[-1]['d9']} d11={lines[-1]['d11']} d0={lines[-1]['d0']} d5={lines[-1]['d5']} {lines[-1]['d3']}")
    print("  scan:", " ".join(f"{v:5.2f}" for v in avg))
if __name__=="__main__": snap()
