import sys, time
def rd(d):
    with open(f'/dev/robot/d{d}') as f:
        return f.readline().strip()
def wr(d, s):
    with open(f'/dev/robot/d{d}','w') as f:
        f.write(s+'\n')
def snap():
    return {d: rd(d) for d in range(5)}
if __name__=='__main__':
    print(snap())
