from drive import *
import math

P0 = prof(scan_pts()); h0 = heading()
print('heading before:', h0)
wheels('2','-2', 3.0)  # d1-d7=+4
h1 = heading(); P1 = prof(scan_pts())
ang,res = est_rotation(P0,P1)
print(f'heading {h0}->{h1} (d={h1-h0:+.1f}); profile shift est {ang:+.0f} deg res={res:.3f}')
stop()
