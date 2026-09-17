s=open('ep3.py').read()
i0=s.index('def follow(dt=0.12):')
i1=s.index('def main():')
new=open('newfollow_body.txt').read()
s=s[:i0]+new+s[i1:]
open('ep3.py','w').write(s)
print('rewritten')
