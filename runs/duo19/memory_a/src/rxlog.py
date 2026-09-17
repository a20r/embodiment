import rio, time
f=open('/bot/src/rx.log','a')
while True:
    m=rio.rd('d10',0.5)
    if m: f.write('%s | %s | d11=%s\n'%(time.strftime('%H:%M:%S'), m, rio.rd('d11'))); f.flush()
    time.sleep(0.3)
