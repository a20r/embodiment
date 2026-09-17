import sys, time
sys.path.insert(0,'/bot/src')
from rio import write
def drive(l, r, secs):
    write('d1', str(l)); write('d7', str(r))
    time.sleep(secs)
    write('d1','0'); write('d7','0')
if __name__=="__main__":
    drive(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
