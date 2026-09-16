import time, sys, os
sys.path.insert(0,"/bot/src")
seen=0
while True:
    try:
        lines=open("/bot/src/rx.log").readlines()[-30:]
        for ln in lines:
            if "A HERE" in ln or ("RX" in ln and ("here=1" in ln or "goal=1" in ln)):
                os.system("/bot/src/killhelper.sh explore6.py")
                if not os.path.exists("/bot/src/HOMEMODE"):
                    open("/bot/src/HOMEMODE","w").write("homing on A\n")
                    os.system("nohup python3 /bot/src/homeA.py > /bot/src/homeA.out 2>&1 &")
    except Exception as e:
        pass
    time.sleep(3)
