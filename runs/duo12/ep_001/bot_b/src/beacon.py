import time, threading, json, os, subprocess
def get_pose():
    try:
        line=subprocess.check_output(['tail','-1','/bot/src/map.log']).decode()
        p=json.loads(line)
        return {'x':p['x'],'y':p['y'],'d11':p['d11'],'here':p['st'].get('here',0)}
    except Exception: return {}
def listen():
    while True:
        try:
            with open('/dev/robot/d10') as f:
                for line in f:
                    line=line.strip()
                    if line:
                        with open('/bot/src/rx.log','a') as g:
                            g.write(f"{time.time():.1f} {line}\n")
        except Exception:
            time.sleep(0.5)
threading.Thread(target=listen,daemon=True).start()
i=0
while True:
    p=get_pose()
    msg=f"botA PING x={p.get('x','?')} y={p.get('y','?')} d11={p.get('d11','?')} here={p.get('here','?')} seq={i}"
    extra=[]
    try:
        if os.path.exists('/bot/src/tx_queue.txt'):
            with open('/bot/src/tx_queue.txt') as f: extra=[l.strip() for l in f if l.strip()]
            os.remove('/bot/src/tx_queue.txt')
    except Exception: pass
    try:
        with open('/dev/robot/d8','w') as f:
            f.write(msg+'\n')
            for e in extra: f.write(e+'\n')
    except Exception: pass
    i+=1
    time.sleep(4)
