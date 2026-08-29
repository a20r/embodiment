import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time

def hdg(): 
    try: return float(read_port("d1"))
    except: return None

combos = [("d10","1.0"),("d10","-1.0"),("d11","1.0"),("d11","-1.0")]
for port,val in combos:
    h0=hdg(); l0=lidar()
    end=time.time()+2.5
    while time.time()<end:
        write_port(port,val); time.sleep(0.05)
    h1=hdg(); l1=lidar()
    print(f"{port}={val}: hdg {h0}->{h1}  dl={[round(b-a,2) for a,b in zip(l0,l1)]}", flush=True)
    # stop
    write_port(port,"0")
    print("  d2=",read_port("d2"),"d5=",read_port("d5"),"d6=",read_port("d6"),"d7=",read_port("d7"),"d8=",read_port("d8"),"d9=",read_port("d9"), flush=True)
