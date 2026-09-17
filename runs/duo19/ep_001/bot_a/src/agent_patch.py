e=open('/bot/src/explore2.py').read()
# goal handling: stay on goal and broadcast forever
e=e.replace("""    if 'goal=0' not in (rec['d3'] or 'goal=0'): print('GOAL'); log('GOAL'); break""",
"""    if 'goal=0' not in (rec['d3'] or 'goal=0'):
        ctl.stop(); log('GOAL'); open('/bot/src/tx_msg.txt','w').write('GOAL AT (%.1f,%.1f) in my frame. A is ON THE GOAL, stopped, waiting for you. Home in on rising d11 (signal). Tell me when you arrive.\\n'%(x,y))
        while True:
            time.sleep(5); r=log('ONGOAL'); 
            if 'goal=0' in (r['d3'] or ''): break
        continue""")
# homing mode: when rx contains GOAL AT, prefer options by d11 memory
e=e.replace("lasttx=0; steps=0; visited={}","lasttx=0; steps=0; visited={}; homing=False; sig={}")
e=e.replace("""    if time.time()-lasttx>20:""","""    if rec.get('rx') and 'GOAL AT' in rec['rx'].upper() and not homing:
        homing=True; log('HOMING-ON',None,rec['rx'])
    try: sig[(round(x/CELL),round(y/CELL))]=float(rec['d11'])
    except: pass
    if time.time()-lasttx>20:""")
e=e.replace("""        score+=min(dist,2.5)*0.3
        opts.append((score,name,ang))""","""        score+=min(dist,2.5)*0.3
        if homing:
            here=sig.get((round(x/CELL),round(y/CELL)),0.5); there=sig.get(cell)
            score = (there-here)*30 if there is not None else 2.0+(0.5 if name=='F' else 0)
            if name=='B': score-=1.0
        opts.append((score,name,ang))""")
open('/bot/src/explore2.py','w').write(e); print('ok', e.count('homing'))
