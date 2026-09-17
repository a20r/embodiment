import sys,time
sys.path.insert(0,"/bot/src")
from robot import send, read
import json
n=0
while True:
    n+=1
    send(f"A PING {n} pos-unknown")
    time.sleep(20)
