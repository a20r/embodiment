import time, threading, sys
def listen():
    while True:
        try:
            with open('/dev/robot/d10') as f:
                for line in f:
                    line=line.strip()
                    if line:
                        with open('/bot/src/rx.log','a') as g:
                            g.write(f"{time.time():.1f} {line}\n")
        except Exception as e:
            time.sleep(0.5)
threading.Thread(target=listen,daemon=True).start()
i=0
while True:
    try:
        with open('/dev/robot/d8','w') as f:
            f.write(f"HELLO from botA seq={i}\n")
    except Exception: pass
    i+=1
    time.sleep(3)
