import threading, time, json
store = {}
def reader(d):
    path=f"/dev/robot/d{d}"
    while True:
        try:
            with open(path) as f:
                line = f.readline()
            store[f"d{d}"] = line.strip()
        except Exception as e:
            time.sleep(0.5)
for d in [0,2,4,5,6,7]:
    threading.Thread(target=reader, args=(d,), daemon=True).start()
while True:
    time.sleep(0.3)
    print(json.dumps(dict(sorted(store.items()))), flush=True)
