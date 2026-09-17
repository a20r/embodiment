import time, json
from rob import *
with open('/tmp/tel.log','a') as out:
    while True:
        try:
            d={'t':round(time.time(),2),'h':heading(),'e':enc(),'r':ranges(),
               'd2':rd('d2'),'d9':rd('d9'),'s':status()}
            out.write(json.dumps(d)+'\n'); out.flush()
        except Exception as ex:
            out.write('ERR '+str(ex)+'\n'); out.flush()
        time.sleep(0.4)
