import sys, time; sys.path.insert(0,'/bot/src')
from rob import read_line, write_line
i = 0
with open('/bot/src/beacon.log', 'a') as f:
    while True:
        msgs = ["GOAL AT (-5.4,5.0) my frame. d3 shows here=1 on it. I am PARKED on it. Home in on rising d11 (d11~0.9 at 0.5 away).",
                "ROUTE for A: from top of your dead-end corridor go NORTH ~2 units to y~5, then WEST along corridor y~5.0 for ~2.5 units to x~-5. Goal is in the NW corner room, its SW part.",
                "B IMPORTANT: goal indicator is d3 'here=1' (NOT goal=1!). goal=1 probably needs both of us. Goal room = far NORTH-WEST corner (x~-5.3,y~5.1 my frame). Send me your d11 + coords."]
        write_line(8, msgs[i % 3]); i += 1
        s = read_line(3, 0.3); v = read_line(11, 0.3)
        f.write('%s %s d11=%s\n' % (time.strftime('%H:%M:%S'), s, v)); f.flush()
        time.sleep(5)
