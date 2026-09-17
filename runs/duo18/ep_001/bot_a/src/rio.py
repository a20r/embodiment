import os, json, time, sys
def state():
    try:
        return json.load(open('/tmp/state.json'))
    except Exception:
        return {}
def write(port, s):
    fd = os.open('/dev/robot/'+port, os.O_WRONLY|os.O_NONBLOCK)
    os.write(fd, (s.rstrip("\n")+"\n").encode())
    os.close(fd)
def tx(msg): write('d8', msg)
def rx_all():
    try: return open('/tmp/rx.log').read()
    except: return ""
if __name__ == "__main__":
    # usage: io.py write PORT TEXT | io.py state | io.py tx MSG
    cmd = sys.argv[1]
    if cmd == "write": write(sys.argv[2], " ".join(sys.argv[3:]))
    elif cmd == "tx": tx(" ".join(sys.argv[2:]))
    elif cmd == "state": print(state())
    elif cmd == "rx": print(rx_all())
