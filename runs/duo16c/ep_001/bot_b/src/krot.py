from drive import *
e0=enc(); h0=heading()
wheels('5','-5',3.0)
e1=enc(); h1=heading()
print('enc d6,d9 delta:', e1[0]-e0[0], e1[1]-e0[1], 'd4:', h0,'->',h1)
stop()
