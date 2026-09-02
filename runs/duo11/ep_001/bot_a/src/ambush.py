import rob2 as R, time, json, sys

LOG=open('/memory/ambush.log','a')
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n'); LOG.flush()

R.stop()
tend=time.time()+(float(sys.argv[1]) if len(sys.argv)>1 else 2000)
n=0
while time.time()<tend:
    n+=1
    R.tx("PING B x=0.00 y=0.00")
    if n%3==0: R.tx("GOAL?")
    v=R.sig(); st=R.status()
    if v>0.15 or 'here=1' in st or 'goal=1' in st:
        log(sig=round(v,3), st=st)
    time.sleep(0.8)
R.stop()
