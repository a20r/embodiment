import time
from rd import rd
for i in range(14):
    t=time.time()
    print(f'd0={rd("d0")} d1={rd("d1")} d2={rd("d2")} d3={rd("d3")} d6={rd("d6")}')
    print('   d5',rd('d5'))
    print('   d8',rd('d8'))
    time.sleep(0.55)
