import time, subprocess

def rd(p):
    r = subprocess.run(['timeout','1','cat',f'/dev/robot/{p}'], capture_output=True, text=True)
    return r.stdout.strip()

def wr(p, s):
    subprocess.run(['timeout','1','bash','-c',f"echo '{s}' > /dev/robot/{p}"], capture_output=True)

print('before d0', rd('d0'), 'd3', rd('d3'), 'd4', rd('d4'), 'd5', rd('d5'), 'd6', rd('d6'), 'd9', rd('d9'))
wr('d1', '0.5')
time.sleep(2)
print('after d1=0.5 d0', rd('d0'), 'd3', rd('d3'), 'd4', rd('d4'), 'd5', rd('d5'), 'd6', rd('d6'), 'd9', rd('d9'))
time.sleep(2)
print('after 2s d0', rd('d0'), 'd3', rd('d3'), 'd4', rd('d4'), 'd5', rd('d5'), 'd6', rd('d6'), 'd9', rd('d9'))
wr('d1', '0')
time.sleep(1)
print('after stop d0', rd('d0'), 'd3', rd('d3'), 'd4', rd('d4'), 'd5', rd('d5'), 'd6', rd('d6'), 'd9', rd('d9'))
