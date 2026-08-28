import lib, time
for s in (60,100,200):
    e0=int(lib.read("d7")); t0=time.time()
    lib.wheels(s,s); time.sleep(2); lib.stop()
    e1=int(lib.read("d7")); dt=time.time()-t0
    print(f"cmd {s}: {(e1-e0)/dt/700:.2f} units/s",flush=True)
    lib.wheels(-30,-30); time.sleep(1.5); lib.stop()
