import lib, time
def ints():
    return {p: lib.read(p) for p in ["d4","d7","d8","d9"]}
print("static:", ints())
lib.wheels(20,20)
for i in range(4):
    time.sleep(1); print("moving:", ints())
lib.stop()
print("stopped:", ints())
l=lib.lidar(); print("lidar:", l)
