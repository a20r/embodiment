#!/bin/bash
echo "cells:$(python3 -c "import json;d=json.load(open('/memory/grid.json'));print(len(d['visited']),d['pos'])" 2>/dev/null)"
grep -v RX /memory/grid.log | tail -2
echo "rx:$(grep -c 'RX' /memory/grid.log) goal:$(grep -ci goalflag /memory/grid.log) d0:$(grep -c 'D0 ' /memory/grid.log) alive:$(pgrep -fc 'u [a]gent')"
