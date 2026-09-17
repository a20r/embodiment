import rb, time, statistics
from ctl import Bot
def mscan(b, n=7):
    cols=[[] for _ in range(16)]
    for _ in range(n):
        s=b.scan()
        if s:
            for k,v in enumerate(s):
                if v>0: cols[k].append(v)
        time.sleep(0.05)
    return [round(statistics.median(c),2) if c else -1 for c in cols]
def show(b):
    s=mscan(b); h=b.heading()
    names={0:'E',45:'NE',90:'N',135:'NW',180:'W',225:'SW',270:'S',315:'SE'}
    out=[]
    for k,v in enumerate(s):
        a=(h+22.5*k)%360
        out.append('%d:%s'%(round(a),v))
    print('h=%.0f'%h, ' '.join(out))
    return s,h
if __name__=='__main__':
    b=Bot(); show(b); print(rb.rd('d3'), rb.rd('d11'))
