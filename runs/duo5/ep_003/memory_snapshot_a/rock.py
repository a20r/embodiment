import time
D='/dev/robot/'
def mot(l,r):
    open(D+'d10','w').write(f"{l}\n"); open(D+'d11','w').write(f"{r}\n")
while True:
    mot(42,42); time.sleep(0.6)
    mot(-42,-42); time.sleep(0.6)
