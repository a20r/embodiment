#!/bin/bash
echo "Doing a long blocking read on d10..."
timeout 5 cat /dev/robot/d10
echo "Done"
