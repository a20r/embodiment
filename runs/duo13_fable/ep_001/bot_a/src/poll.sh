#!/bin/bash
# compact progress: last N AT lines: time pos open sig goal
f=${1:-/tmp/explore2.out}; n=${2:-4}
grep -E "AT|failed|exhausted|GOAL|blocked" $f | tail -n $n | sed -E "s/view=\{[^}]*\} //; s/status=\{'tick': '[0-9]+', //; s/h=[0-9]+ //; s/'//g" | cut -c1-90
grep -c GOAL $f | sed 's/^/goalhits=/'
tail -n 1 /tmp/rx.txt | cut -c1-200
