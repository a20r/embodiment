import sys, time, json, os, select
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/radio2.log','a')
def L(s):
    LOG.write('%.2f %s\n'%(time.time()%100000,s)); LOG.flush()

def state():
    s=R.status()
    # like 'tick=... goal=0 here=0 tx=372:busy'
    for tok in s.split():
        if tok.startswith('tx='):
            return tok[3:]
    return ''

def main():
    L('responder start')
    seq=0
    idle_since=None
    while True:
        msg=R.rx()
        if msg:
            L('RX '+msg.replace('\n',' | '))
        st=state()
        now=time.time()
        if st.endswith('idle') or st.endswith('lost'):
            if idle_since is None: idle_since=now
        else:
            idle_since=None
        # reply when channel idle for >=0.4s and we've heard them recently
        if idle_since and now-idle_since>=0.4:
            seq+=1
            ok=R.tx(f'ROBOT-B here, I copy you! Over.')
            L('TX reply %d ok=%s state=%s'%(seq,ok,st))
            idle_since=None
        time.sleep(0.05)

if __name__=='__main__':
    try: main()
    finally: pass
