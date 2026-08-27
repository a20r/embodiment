import os, time

def rd(p):
    fd = os.open(f"/dev/robot/d{p}", os.O_RDONLY | os.O_NONBLOCK)
    try:
        time.sleep(0.15)
        data = os.read(fd, 4096).decode()
        return data.strip().split("\n")[-1]
    except BlockingIOError:
        return "<empty>"
    finally:
        os.close(fd)

def wr(p, s):
    fd = os.open(f"/dev/robot/d{p}", os.O_WRONLY | os.O_NONBLOCK)
    os.write(fd, (s+"\n").encode())
    os.close(fd)

if __name__ == "__main__":
    for i in range(9):
        try:
            print(i, rd(i))
        except Exception as e:
            print(i, "ERR", e)
