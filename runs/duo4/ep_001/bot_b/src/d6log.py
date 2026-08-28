import time
with open("/memory/d6.log","a",buffering=1) as out:
    while True:
        try:
            with open("/dev/robot/d6") as f:
                line=f.readline().strip()
            out.write(f"{time.time():.1f} {line}\n")
        except Exception as e:
            out.write(f"{time.time():.1f} ERR {e}\n"); time.sleep(1)
