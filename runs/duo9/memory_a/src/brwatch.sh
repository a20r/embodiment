#!/bin/bash
while true; do
  B=$(grep -o 'bearing [0-9]* deg' /memory/radio_rx.log | tail -1 | grep -o '[0-9]*')
  [ -n "$B" ] && echo $B > /memory/br.txt
  sleep 8
done
