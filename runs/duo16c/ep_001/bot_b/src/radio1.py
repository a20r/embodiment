import os, time, subprocess

def tx(msg):
    for _ in range(3):
        fd=os.open('/dev/robot/d8', os.O_WRONLY|os.O_NONBLOCK)
        try:
            os.write(fd, (msg+'\n').encode()); os.close(fd); return True
        except Exception as e:
            os.close(fd); time.sleep(0.1)
    return False

def rx(timeout=1.0):
    r = subprocess.run(['timeout',str(timeout),'cat','/dev/robot/d10'], capture_output=True, text=True)
    return r.stdout.strip()

print('tx ok:', tx('HELLO from robot A'))
for i in range(8):
    m = rx(0.8)
    print(f'rx[{i}]: {m!r}')
    if m: tx('ACK '+m[:20])
    time.sleep(0.3)
tx('HELLO are you there?')
for i in range(5):
    print(f'rx2[{i}]: {rx(0.8)!r}')
