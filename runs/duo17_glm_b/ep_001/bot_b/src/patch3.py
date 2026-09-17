s=open('ep3.py').read()
s=s.replace("""        if f<0.24:
            right=min(b(10),b(11),b(12),b(13)); left=min(b(3),b(4),b(5),b(6))
            steer=1.0 if right>left else -1.0
            base=11""","""        if f<0.24:
            if time.time()>ESC['until']:
                right=min(b(10),b(11),b(12),b(13)); left=min(b(3),b(4),b(5),b(6))
                d=1 if right>left else -1
                L('ESCAPE dir=%d f=%.2f L=%.2f R=%.2f'%(d,f,left,right))
                R.motors(-13,-13); time.sleep(0.6)
                if d>0: R.motors(20,-10)
                else: R.motors(-10,20)
                time.sleep(1.3)
                R.stop()
                ESC['until']=time.time()+2.0
                continue""")
s=s.replace("random.seed(7)","random.seed(7)\nESC={'until':0.0}")
open('ep3.py','w').write(s)
print('patched')
