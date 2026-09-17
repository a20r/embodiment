import sys, time
sys.path.insert(0,'/bot/src')
import robot2 as R
# background beacon: 1Hz informative burst
msgs=[
 'B-TO-A: I AM HOMING ON YOUR SIGNAL. KEEP TRANSMITTING.',
 'B-TO-A: IF YOU CAN MOVE, STAY PUT OR COME TO ME.',
 'B-TO-A: GOAL STATUS? SEND: GOAL YES/NO, STUCK YES/NO.',
]
i=0
while True:
    try:
        R.tx(msgs[i%len(msgs)])
    except Exception:
        pass
    i+=1
    time.sleep(1.0)
