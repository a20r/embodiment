from drive import *
for v in ['2','5','10','20','50']:
    e0=enc(); wheels(v,v,2.0); e1=enc()
    r6=(e1[0]-e0[0])/2.0; r9=(e1[1]-e0[1])/2.0
    print(f'cmd {v}: d6 {r6:.1f}/s d9 {r9:.1f}/s flags={flags()}')
stop()
