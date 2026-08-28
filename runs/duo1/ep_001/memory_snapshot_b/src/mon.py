import threading, time, sys
def reader(name):
    with open(f"/dev/robot/{name}") as f:
        for line in f:
            print(f"[{time.time():.1f}] {name}: {line.rstrip()}", flush=True)
for n in ["d0","d2","d4","d5","d7","d8","d9","d10"]:
    threading.Thread(target=reader, args=(n,), daemon=True).start()
time.sleep(float(sys.argv[1]) if len(sys.argv)>1 else 10)
