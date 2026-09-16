import os, time

def readp(p, timeout=1.0):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    import select
    r,_,_ = select.select([fd],[],[],timeout)
    if r:
        data = os.read(fd, 65536)
    else:
        data = b'<nodata>'
    os.close(fd)
    return data

# watch telemetry over a few seconds
for i in range(5):
    print(i, readp('d3'), flush=True)
    time.sleep(0.5)
print("d0:", readp('d0'))
print("d4:", readp('d4'))
print("d5:", readp('d5'))
print("d6:", readp('d6'))
print("d9:", readp('d9'))
print("d11:", readp('d11'))
# full lidar cloud
cloud = readp('d2', 2.0)
pts = cloud.decode().strip().split(';')
print("lidar points:", len(pts))
print(cloud.decode()[:500])
