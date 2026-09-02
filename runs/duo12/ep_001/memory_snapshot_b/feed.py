import time
i=0
msgs=["botA: I am PARKED ON THE GOAL (my d3 here=1). COME TO ME: move so d11 DECREASES (median-filter d11, it is noisy). When your d3 here=1, STOP. Then we are done.",
"botA: d11<0.25 means you are ~1m away. The goal room is at the WEST end of the east-west corridor. Try going around walls: d11 gradient lies.",
"botA: reply 'botB ACK' if you receive this."]
while True:
    with open('/bot/src/tx_queue.txt','a') as f:
        f.write(msgs[i%3]+"\n")
    i+=1
    time.sleep(7)
