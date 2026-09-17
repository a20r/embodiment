#!/bin/sh
# cycle bias directions unless mode is stop
while true; do
  for d in 180 90 0 270 135 225 45 315; do
    m=$(cat /bot/mode); case "$m" in stop*) sleep 5; continue;; esac
    echo "bias $d" > /bot/mode; sleep 100
  done
done
