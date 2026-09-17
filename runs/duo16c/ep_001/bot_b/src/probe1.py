import time

def read(p, t=1.0):
    try:
        with open(f'/dev/robot/{p}') as f:
            import selectors
            return f.readline().strip()
    except Exception as e:
        return f'ERR {e}'

def write(p, s):
    try:
        with open(f'/dev/robot/{p}', 'w') as f:
            f.write(s + '\n')
        return 'ok'
    except Exception as e:
        return f'ERR {e}'

# try writing 0 to all writable-looking ports to see which accept
for p in ['d0','d1','d5','d6','d7','d9','d11']:
    print(p, write(p, '0'))
time.sleep(1)
print('d3', read('d3'))
