import time
D='/dev/robot/'
def mot(l,r):
    open(D+'d10','w').write(f"{l}\n"); open(D+'d11','w').write(f"{r}\n")
while True:
    mot(55,55); time.sleep(1.0)
    mot(-55,-55); time.sleep(1.0)
