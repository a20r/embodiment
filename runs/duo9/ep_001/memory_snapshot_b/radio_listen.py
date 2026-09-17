import time
while True:
    try:
        with open("/dev/robot/d4") as f:
            line = f.readline().strip()
        if line:
            print(f"{time.time():.1f} RX: {line}", flush=True)
    except Exception as e:
        time.sleep(0.5)
