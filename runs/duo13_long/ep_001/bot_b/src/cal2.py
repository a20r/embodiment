import time, sys
sys.path.insert(0,"/bot/src")
from robot import *

def snap(tag):
    print(tag, "d6=%s d9=%s h=%s d0=%s d5=%s d11=%s d3=%s" % (rd("d6"),rd("d9"),rd("d4"),rd("d0"),rd("d5"),rd("d11"),rd("d3")))

snap("start")
motors(1,1); time.sleep(2); snap("fwd(1,1) 2s")
motors(2,2); time.sleep(2); snap("fwd(2,2) 2s")
motors(0,0); time.sleep(0.5); snap("stop")
motors(-1,-1); time.sleep(2); snap("back(-1,-1) 2s")
motors(0,0); time.sleep(0.5); snap("stop")
motors(1,-1); time.sleep(2); snap("spin(1,-1) 2s")
motors(-1,1); time.sleep(2); snap("spin(-1,1) 2s")
motors(0,0); time.sleep(0.3); snap("stop")
