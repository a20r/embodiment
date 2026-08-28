#!/bin/bash
for p in d0 d2 d4 d7 d8 d9; do echo -n "$p: "; timeout 1 head -n1 /dev/robot/$p; done
