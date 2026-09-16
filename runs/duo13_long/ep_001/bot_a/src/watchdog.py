import time, os, subprocess, json
last_restart=0
def log(msg):
    with open('/bot/src/watchdog.log','a') as f:
        f.write(f'[{time.strftime("%H:%M:%S")}] {msg}\n')
def motors_stopped():
    # append stop commands directly
    for p in ('d1','d7'):
        try:
            fd=os.open('/dev/robot/'+p, os.O_WRONLY|os.O_NONBLOCK)
            os.write(fd, b'0\n'); os.close(fd)
        except Exception: pass
log('watchdog start')
while True:
    time.sleep(30)
    try:
        alive = any('explore8' in open(f'/proc/{p}/cmdline').read().replace('\0',' ')
                    for p in os.listdir('/proc') if p.isdigit() and os.path.exists(f'/proc/{p}/cmdline'))
        mtime = os.path.getmtime('/bot/src/explore8.log')
        age = time.time()-mtime
        if (not alive) or age>150:
            if time.time()-last_restart>120:
                log(f'restart (alive={alive} logage={age:.0f})')
                motors_stopped()
                subprocess.Popen(['python3','/bot/src/explore8.py'],
                                 stdout=open('/bot/src/explore8_out.txt','a'),
                                 stderr=subprocess.STDOUT,
                                 start_new_session=True)
                last_restart=time.time()
    except Exception as e:
        log('err '+repr(e))
