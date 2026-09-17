import os, signal
base='/dev/robot/'
def try_open_w(name, timeout=0.6):
    def handler(s,f): raise TimeoutError()
    signal.signal(signal.SIGALRM, handler)
    signal.setitimer(signal.ITIMER_REAL, timeout)
    try:
        fd = os.open(base+name, os.O_WRONLY)
        os.close(fd)
        return 'WRITABLE (reader present)'
    except TimeoutError:
        return 'no reader'
    except Exception as e:
        return 'err '+str(e)
    finally:
        signal.alarm(0)
for n in ['d0','d1','d2','d3','d4','d5','d6','d7','d8','d9','d10','d11']:
    print(n, try_open_w(n))
