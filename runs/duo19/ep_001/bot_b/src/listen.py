import sys, time
sys.path.insert(0, '/bot/src')
from rob import read_line
with open('/bot/src/radio.log', 'a') as f:
    while True:
        l = read_line(10, 2.0)
        if l is not None and l != '':
            f.write('%s %s\n' % (time.strftime('%H:%M:%S'), l)); f.flush()
        time.sleep(0.2)
