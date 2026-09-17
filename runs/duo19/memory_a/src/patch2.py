e=open('/bot/src/explore2.py').read()
e=e.replace("""    if r>0.75: ctl.turn(-90); log('turnR')
    elif f>0.45: pass
    elif l>0.75: ctl.turn(90); log('turnL')
    else: ctl.turn(180); log('turnBack')""",
"""    HAND=sys.argv[4] if len(sys.argv)>4 else 'R'
    if HAND=='R':
        if r>0.75: ctl.turn(-90); log('turnR')
        elif f>0.45: pass
        elif l>0.75: ctl.turn(90); log('turnL')
        else: ctl.turn(180); log('turnBack')
    else:
        if l>0.75: ctl.turn(90); log('turnL')
        elif f>0.45: pass
        elif r>0.75: ctl.turn(-90); log('turnR')
        else: ctl.turn(180); log('turnBack')""")
open('/bot/src/explore2.py','w').write(e)
print('ok', 'HAND' in e)
