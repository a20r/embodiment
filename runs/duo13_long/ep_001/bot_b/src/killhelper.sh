#!/bin/bash
# usage: killhelper.sh <pattern> — kills all matching except own ancestors
self=$$
ancestors=""
p=$self
while [ "$p" != "1" ] && [ -n "$p" ]; do
  ancestors="$ancestors $p"
  p=$(awk '{print $4}' /proc/$p/stat 2>/dev/null)
done
for q in $(pgrep -f "$1"); do
  skip=0
  for a in $ancestors; do [ "$q" = "$a" ] && skip=1; done
  [ $skip -eq 0 ] && kill "$q" 2>/dev/null
done
