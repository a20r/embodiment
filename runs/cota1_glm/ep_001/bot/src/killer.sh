#!/bin/bash
# targeted kill: killer.sh <name>  (matches python3 .*<name>.py only)
for pid in $(ls /proc | grep -E '^[0-9]+$'); do
  c=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null)
  case "$c" in
    *python3*"$1".py*) echo "kill $pid: $c"; kill $pid;;
  esac
done
