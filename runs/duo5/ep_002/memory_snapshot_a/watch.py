import time,re
def d5():
    try:
        s=open('/tmp/state.txt').read()
        m=re.search(r'd5=([0-9.]+)',s)
        return float(m.group(1))
    except: return 0.0
def rxtail():
    try: return open('/tmp/radio.log').read()[-2000:]
    except: return ''
frozen=0
while True:
    v=d5()
    r=rxtail()
    if 'STOP TEST' in r.split('TX')[-1] if False else False: pass
    trig = v>1.3
    if trig and time.time()-frozen>90:
        frozen=time.time()
        open('/tmp/cmd','w').write('stop\n')
        with open('/dev/robot/d0','w') as f: f.write(f'B STOP TEST freeze. my d5={v:.2f}\n')
        with open('/tmp/alert.log','a') as f: f.write(f'{time.time():.0f} FREEZE d5={v:.2f}\n')
        time.sleep(45)  # stay frozen, then resume
        open('/tmp/cmd','w').write('brain2\n')
    time.sleep(1)
