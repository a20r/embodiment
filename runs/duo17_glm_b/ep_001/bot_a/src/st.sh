#!/bin/bash
echo "== $(date +%H:%M) =="
grep -E "^LEG|CONTACT|GOAL|D5|PHASE|turn" /memory/ep6.log | tail -4
grep "^RX" /memory/ep6.log | tail -2
tail -1 /memory/ep6.log
