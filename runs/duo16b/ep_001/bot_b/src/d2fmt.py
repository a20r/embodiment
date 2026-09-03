import re
for fn in ['/memory/d2static.txt','/memory/d2frame.txt']:
    s=open(fn).read()
    print(fn, "len", len(s), "newlines", s.count('\n'), "semis", s.count(';'))
    stripped=re.sub(r'\s+','',s)
    nums=re.findall(r'-?\d*\.?\d+', stripped)
    segs=[x for x in stripped.split(';') if x]
    bad=sum(1 for x in segs if len(re.findall(r'-?\d*\.?\d+',x))!=3)
    print("  numbers:",len(nums), "mod3:", len(nums)%3, "segs:",len(segs),"badsegs:",bad)
    # hypothesis: newline replaced ';'
    s2=s.replace('\n',';')
    segs2=[x for x in s2.split(';') if x]
    bad2=sum(1 for x in segs2 if len(re.findall(r'-?\d*\.?\d+',x))!=3)
    print("  newline-as-semicolon: segs:",len(segs2),"bad:",bad2)
    # hypothesis: newline is noise -> strip; count adjacent merges
    print("  sample stripped head:", stripped[:80])
