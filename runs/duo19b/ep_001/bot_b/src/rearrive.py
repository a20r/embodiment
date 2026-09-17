import sys; sys.path.insert(0,'/bot/src')
import ctl, rio, time, json
off=json.load(open('/bot/off.json')).get('off_hdg') or 190
back=(off+180)%360
ctl.turn_to(back, tol=5); t0=time.time()
while time.time()-t0<20:
    if ctl.status().get('here')==1: break
    s=ctl.scan()
    if s and min(v for v in (s[0],s[1],s[15]) if v>0)<0.13: break
    ctl.drive(45,45); time.sleep(0.1)
ctl.stop(); st=ctl.status(); print(time.strftime('%H:%M:%S'), 'back on:', st)
rio.write_line('d8', f"A: re-entered: my d3 here={st.get('here')} goal={st.get('goal')}.")
for i in range(5): time.sleep(2); print(ctl.status(), rio.read_line('d11',1))
