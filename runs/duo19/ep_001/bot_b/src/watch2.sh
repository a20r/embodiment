#!/bin/bash
sleep ${1:-55}
echo "[$(date +%H:%M:%S)] $(tail -n 1 /bot/src/beacon.log)"
grep -v "A10:\|A13:\|A14:\|PROTOCOL\|A1:\|A2:\|A3:\|A4:\|A5:\|A6:\|A7:\|A9:" /bot/src/radio.log | tail -n 3 | cut -c1-250
