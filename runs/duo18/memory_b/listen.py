#!/usr/bin/env python3
import os, time, sys
LOG = "/bot/rx.log"
while True:
    try:
        with open("/dev/robot/d10", "r") as f:
            line = f.readline()
        if line:
            with open(LOG, "a") as g:
                g.write("%s %s\n" % (time.strftime("%H:%M:%S"), line.rstrip("\n")))
        else:
            time.sleep(0.05)
    except Exception as e:
        with open(LOG, "a") as g:
            g.write("%s ERR %r\n" % (time.strftime("%H:%M:%S"), e))
        time.sleep(0.5)
