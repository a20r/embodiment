import rb, time, math, json
CPU=2300.0  # encoder counts per range-unit (approx)
class Bot:
    def __init__(self):
        self.x=0.0; self.y=0.0
        self.eA,self.eB=self.enc()
        self.h=self.heading()
        self.log=open('/bot/src/pose.jsonl','a')
    def enc(self):
        for _ in range(3):
            try: return int(rb.rd('d9',0.3)), int(rb.rd('d6',0.3))
            except: pass
        return self.eA,self.eB
    def heading(self):
        for _ in range(3):
            try: return float(rb.rd('d4',0.3))
            except: pass
        return self.h
    def bump(self):
        return rb.rd('d5',0.3)=='1'
    def scan(self):
        for _ in range(3):
            s=rb.rd('d2',0.3).split(',')
            if len(s)==16:
                try: return [float(v) for v in s]
                except: pass
        return None
    def update(self):
        a,b=self.enc(); h=self.heading()
        dA=a-self.eA; dB=b-self.eB
        d=(dA+dB)/2/CPU
        hm=math.radians((self.h+h)/2 if abs(h-self.h)<180 else h)
        # heading increases in beam-index direction; choose x=cos,y=sin in heading frame
        self.x+=d*math.cos(hm); self.y+=d*math.sin(hm)
        self.eA,self.eB=a,b; self.h=h
        return d
    def set(self,vA,vB):
        rb.wr('d1',str(vA)); rb.wr('d7',str(vB))
    def stop(self):
        for _ in range(2): self.set(0,0)
        time.sleep(0.15); self.update()
    def record(self,tag=''):
        s=self.scan()
        rec={'t':round(time.time(),2),'x':round(self.x,3),'y':round(self.y,3),'h':self.h,'scan':s,'d11':rb.rd('d11',0.3),'tag':tag}
        self.log.write(json.dumps(rec)+'\n'); self.log.flush()
        return rec
    def turn_to(self,target,tol=4,speed=30):
        t0=time.time()
        while time.time()-t0<8:
            self.update()
            err=(target-self.h+180)%360-180
            if abs(err)<=tol: break
            sp=speed if abs(err)>25 else 18
            if err>0: self.set(sp,-sp)   # d1 + increases heading
            else: self.set(-sp,sp)
            time.sleep(0.05)
        self.stop()
        return self.h
    def forward(self,dist,speed=50,minfront=0.15,timeout=15):
        """drive forward dist units, stop if bumper or front obstacle"""
        x0,y0=self.x,self.y; t0=time.time(); reason='done'
        h0=self.h
        while time.time()-t0<timeout:
            self.update()
            trav=math.hypot(self.x-x0,self.y-y0)
            if trav>=dist: break
            if self.bump(): reason='bump'; break
            s=self.scan()
            if s:
                front=[v for v in (s[0],) if v>0]+[v for v in (s[1],s[15]) if 0<v<minfront*0.6]
                if front and min(front)<minfront: reason='obstacle %.2f'%min(front); break
            # heading hold
            err=(h0-self.h+180)%360-180
            if s and 0<s[4]<0.6 and 0<s[12]<0.6:
                err+=max(-12,min(12,(s[4]-s[12])*50))
            elif s and 0<s[12]<0.13: err+=8
            elif s and 0<s[4]<0.13: err-=8
            corr=max(-18,min(18,err*1.2))
            sp=speed if dist-trav>0.15 else 25
            self.set(sp+corr, sp-corr)
            time.sleep(0.05)
        else: reason='timeout'
        self.stop()
        return reason, math.hypot(self.x-x0,self.y-y0)
