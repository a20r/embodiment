import lib, time
while lib.read("d5").strip(): pass
lib.wheels(15,15)
for m in ["ping","ping","hello","123","where","goal","help","ping","ping","ping"]:
    lib.write("d3",m); time.sleep(0.25)
    r=lib.read("d5").strip()
    print(f"{m} -> {r}",flush=True)
lib.stop()
# now stationary
for m in ["ping","ping"]:
    lib.write("d3",m); time.sleep(0.25)
    r=lib.read("d5").strip()
    print(f"static {m} -> {r}",flush=True)
