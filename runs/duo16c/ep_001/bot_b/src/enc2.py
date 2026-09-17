import time
from scene import read_line, stream

def enc():
    return float(read_line('d6',0.5) or 0), float(read_line('d9',0.5) or 0)

def rate(dur=2.0):
    a=enc(); time.sleep(dur); b=enc()
    return round((b[0]-a[0])/dur,2), round((b[1]-a[1])/dur,2)

print('base:', rate())
for fmt, v1, v7 in [('plain','1.0','1.0'), ('plain-','1.0','-1.0'), ('plainL','1.0','0.0'), ('plainR','0.0','1.0')]:
    stream(v1, v7, 3.0)
    print(f'{fmt} {v1},{v7}: rate during+after:', rate())
    print(f'{fmt} continued:', rate())
