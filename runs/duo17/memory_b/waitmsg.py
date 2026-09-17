import rb, time, sys
t=float(sys.argv[1]) if len(sys.argv)>1 else 50
t0=time.time(); seen=set()
while time.time()-t0<t:
    m=rb.rd('d10',0.5)
    if m and m not in seen:
        seen.add(m); print(round(time.time()-t0,1), m[:400], flush=True)
        open('msgs.txt','a').write('%d RX: %s\n'%(time.time(),m))
    time.sleep(0.3)
print('status', rb.rd('d3'), 'd11', rb.rd('d11'))
