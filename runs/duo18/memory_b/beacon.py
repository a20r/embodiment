#!/usr/bin/env python3
import rio, time, json
i = 0
while True:
    try:
        p = json.load(open("/bot/pose.txt")); extra = f" d11={p.get('d11')} goal={p.get('goal')}."
    except Exception: extra = ""
    try: extra2 = open("/bot/say.txt").read().strip()
    except Exception: extra2 = ""
    msg = f"B#{i}:{extra2}{extra}"
    rio.write_port(8, msg[:250]); i += 1; time.sleep(6)
