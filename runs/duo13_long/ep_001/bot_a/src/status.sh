#!/bin/bash
echo "== $(date +%H:%M:%S)"
tail -2 /bot/src/explore8.log
python3 -c "
import json
try: print('fingerprints:', len(json.load(open('/bot/src/fp.json'))))
except Exception as e: print('fps err')"
cat /memory/rx_log.txt 2>/dev/null | tail -3
cat /memory/goal_found.json 2>/dev/null
