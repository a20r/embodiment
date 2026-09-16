import subprocess

for p in ['d0','d1','d5','d6','d7','d9','d11']:
    r = subprocess.run(['timeout','1','bash','-c',f'echo 0 > /dev/robot/{p}'], capture_output=True, text=True)
    print(p, r.returncode, r.stderr.strip()[:80])
