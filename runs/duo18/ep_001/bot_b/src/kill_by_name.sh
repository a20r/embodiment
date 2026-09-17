#!/bin/sh
# usage: kill_by_name.sh NAME  -- kills python3 processes whose cmdline contains NAME (never touches shells)
for p in $(pgrep -f "$1"); do
  if grep -q "python3\|cycle" /proc/$p/cmdline 2>/dev/null && ! grep -q "bash" /proc/$p/cmdline 2>/dev/null; then kill $p && echo "killed $p"; fi
done
