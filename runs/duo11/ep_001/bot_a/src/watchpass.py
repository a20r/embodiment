import rob2 as R, time, json
LOG=open('/memory/pass.log','a')
R.stop()
t0=time.time()
while time.time()-t0<150:
    v=R.sig(); s=R.scan(); h=R.heading()
    R.tx("PING B x=0.00 y=0.00")
    LOG.write(json.dumps({'t':round(time.time(),1),'sig':round(v,3),'h':round(h,1),
       'scan':[round(x,2) for x in s]})+'\n'); LOG.flush()
    time.sleep(0.25)
