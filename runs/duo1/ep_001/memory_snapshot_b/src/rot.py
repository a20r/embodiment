import lib, time
while lib.read("d5").strip(): pass
lib.wheels(-12,12)
for i in range(24):
    lib.write("d3","ping"); time.sleep(0.25)
    r=lib.read("d5").strip()
    print(f"h={lib.heading():6.1f} v={r}",flush=True)
lib.stop()
