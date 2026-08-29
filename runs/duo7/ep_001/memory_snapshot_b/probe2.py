import time
def rd(p):
    with open('/dev/robot/'+p) as f: return f.readline().strip()
def wr(p,v):
    with open('/dev/robot/'+p,'w') as f: f.write(str(v)+'\n')
# stop
wr('d10',0); wr('d11',0)
print('h',rd('d1'),'lidar',rd('d3'))
# single write big, then observe passive
wr('d10',100); wr('d11',100)
for i in range(6):
    time.sleep(0.5)
    print(i, 'h',rd('d1'), 'l', rd('d3'))
wr('d10',0); wr('d11',0)
print('stopped')
print('d6',rd('d6'),'d2',rd('d2'),'d7',rd('d7'),'d8',rd('d8'),'d9',rd('d9'))
