#!/bin/bash
cd /bot/src
rm -f /tmp/nav.err
setsid python3 nav2.py >>/tmp/nav.err 2>&1 </dev/null &
