src=open('/bot/src/robot.py').read()
a="""def heading():
    s=rd("d4")
    try: return float(s)
    except: return None"""
b="""_lasth=[180.0]
def heading():
    s=rd("d4")
    try:
        v=float(s)
        _lasth[0]=v
        return v
    except:
        return _lasth[0]"""
src=src.replace(a,b)
open('/bot/src/robot.py','w').write(src)
import py_compile; py_compile.compile('/bot/src/robot.py', doraise=True); print("robot.py patched")
