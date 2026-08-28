import lib, time
for i in range(6):
    l=lib.lidar()
    print(f"t{i} f={l[0]:.2f} b={l[8]:.2f} l={l[4]:.2f} r={l[12]:.2f}")
    if i==0: lib.wheels(20,20)
    time.sleep(1)
lib.stop()
