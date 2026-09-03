#!/bin/bash
cd /bot/src
pkill -f 'ctrl2.py'; pkill -f 'goto.py'; pkill -f 'leader.py'; pkill -f 'explore2.py'
sleep 0.4
echo 0 > /dev/robot/d10; echo 0 > /dev/robot/d11
case "$1" in
  approach) nohup python3 ctrl2.py > ctrl2.out 2>&1 & ;;
  lead)     nohup python3 leader.py > leader.out 2>&1 & ;;
  explore)  nohup python3 explore2.py > explore2.out 2>&1 & ;;
  goto)     nohup python3 goto.py > goto.out 2>&1 & ;;
  stop)     : ;;
esac
echo done-$1
# usage: go.sh explore
