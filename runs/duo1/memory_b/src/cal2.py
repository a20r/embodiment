import lib, time
l=lib.lidar(); print("lidar", l)
j=max(range(16), key=lambda i: l[i])
print("max ray", j, l[j])
delta = ((22.5*j+180)%360)-180
print("turn by", delta)
lib.turn_by(delta)
time.sleep(0.5)
l2=lib.lidar(); print("after", l2, "front", l2[0])
# now drive forward
lib.drive(15,15,2)
l3=lib.lidar(); print("drove", l3, "front", l3[0])
