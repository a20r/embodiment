import rb, time, json, sys
out=open('/bot/src/log.jsonl','a')
msgs=open('/bot/src/msgs.txt','a')
while True:
    t=time.time()
    rec={'t':round(t,2)}
    for p in ['d0','d2','d3','d4','d5','d6','d9','d11']:
        rec[p]=rb.rd(p,0.3)
    m=rb.rd('d10',0.3)
    if m.strip():
        rec['msg']=m
        msgs.write(f'{t:.1f} RX: {m}\n'); msgs.flush()
    out.write(json.dumps(rec)+'\n'); out.flush()
    time.sleep(0.1)
