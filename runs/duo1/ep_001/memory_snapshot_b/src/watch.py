import lib, time
for i in range(40):
    lib.write("d3","ping"); time.sleep(0.5)
    r=lib.read("d5").strip()
    print(f"{time.time():.0f} {r}",flush=True)
