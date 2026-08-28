#!/bin/bash
awk '/AGENT start/{n=0; delete seen} /CELL \(/{split($0,a,/[()]/); if(!(a[2] in seen)){seen[a[2]]=1;n++}} END{print "uniq cells this run:",n}' /memory/grid.log
tail -2 /memory/grid.log
grep -E 'GOALFLAG' /memory/grid.log | tail -3
grep RX /memory/grid.log | tail -3
