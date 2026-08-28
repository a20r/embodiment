import sys, time
sys.path.insert(0,'/bot/src')
from rio import write_port
while True:
    write_port("d8","B ACK plan: goal-finder parks there + sends GOALFOUND repeatedly; other homes on d6 signal. B still exploring, no goal yet.")
    time.sleep(4)
