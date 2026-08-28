import sys, time

def rd(p):
    with open(f"/dev/robot/{p}") as f:
        return f.readline().strip()

def wr(p, v):
    with open(f"/dev/robot/{p}", "w") as f:
        f.write(str(v) + "\n")

def state():
    return rd("d2"), rd("d1"), rd("d9"), rd("d0"), rd("d7")

if __name__ == "__main__":
    d4, d5, dur = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    print("before", state())
    wr("d4", d4); wr("d5", d5)
    t0 = time.time()
    while time.time() - t0 < dur:
        time.sleep(dur/4)
        print(state())
    wr("d4", 0); wr("d5", 0)
    print("after", state())
