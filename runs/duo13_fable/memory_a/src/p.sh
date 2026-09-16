#!/bin/bash
sleep ${1:-58}; echo "$(date +%H:%M) rx=$(wc -l < /tmp/rx.txt) $(tail -n1 /tmp/log.txt | cut -d' ' -f3-) ap=$(tail -n1 /memory/autopilot_log.txt | cut -c1-60)"
