src=open('/bot/src/explore5.py').read()
a="""    dl=min(b[4],b[5]); dr=min(b[11],b[12])
    if dl<0.20: th_t=havg()+7
    elif dl>0.65 and b[1]>0.8: th_t=havg()-7
    elif dr<0.15: th_t=havg()-6"""
b="""    dl=min(b[4],b[5]); dr=min(b[11],b[12])
    if dl>0.95 and b[1]>0.9 and b8>0.6:
        # wide open left: cross over decisively
        log("CROSS left dl=%.2f h=%.0f"%(dl,havg()))
        rotate_to((havg()-50)%360); th_t=None; time.sleep(0.3)
        continue
    if dl<0.20: th_t=havg()+7
    elif dr<0.15: th_t=havg()-6"""
src=src.replace(a,b)
open('/bot/src/explore5.py','w').write(src)
import py_compile; py_compile.compile('/bot/src/explore5.py', doraise=True); print("ok")
