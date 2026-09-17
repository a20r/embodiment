#!/bin/bash
cd /bot && nohup setsid python3 /bot/src/beacon.py > /tmp/beacon.out 2>&1 < /dev/null &
disown; echo beacon started
