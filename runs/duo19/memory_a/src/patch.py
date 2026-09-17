e=open('/bot/src/explore2.py').read()
e=e.replace("x=y=0.0; L0,R0=ctl.enc()","x=float(sys.argv[2]) if len(sys.argv)>2 else 0.0; y=float(sys.argv[3]) if len(sys.argv)>3 else 0.0; L0,R0=ctl.enc()")
e=e.replace("""    f=ctl.front(sc); l=min(v for v in [sc[3],sc[4],sc[5]] if v>=0) if any(v>=0 for v in sc[3:6]) else 9
    r=min(v for v in [sc[11],sc[12],sc[13]] if v>=0) if any(v>=0 for v in sc[11:14]) else 9""",
"""    f=ctl.front(sc); l=sc[4] if sc[4]>=0 else 9; r=sc[12] if sc[12]>=0 else 9
    if 0<=min(sc[3],sc[5])<0.25: l=min(l,0.5)
    if 0<=min(sc[11],sc[13])<0.25: r=min(r,0.5)""")
open('/bot/src/explore2.py','w').write(e)
print('patched', 'sys.argv[2]' in e, 'l=sc[4]' in e)
