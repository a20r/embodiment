import sys, time
def rd(p):
    try:
        with open(f'/dev/robot/d{p}') as f:
            import select
            r,_,_ = select.select([f],[],[],1.0)
            if r: return f.readline().strip()
    except Exception as e: return str(e)
    return ''
for p in [0,1,2,7,9]:
    print(p, rd(p))
