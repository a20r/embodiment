import rio, time
while True:
    try: lines=[l.strip() for l in open('/bot/src/tx_msg.txt') if l.strip()]
    except: lines=[]
    for l in lines:
        rio.wr('d8', l[:240]); time.sleep(2.5)
    time.sleep(8)
