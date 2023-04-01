#!/bin/bash

# Get the PID of the process with the command name "kitti_publisher"
pid=$(pgrep kitti_publisher)

# Loop until the PID is killed
while [ -n "$pid" ]; do
    echo "Getting CPU utilization for PID $pid"
    if test -e "../log/cpu-$pid.log"; then
        top -b -n 1 -p $pid | tail -1 >> ../log/cpu-$pid.log
    else
        touch cpu-$pid.log
        top -b -n 1 -p $pid | tail -1 >> ../log/cpu-$pid.log
    fi
    # Check if the PID still exists
    pid=$(pgrep kitti_publisher)
done

cat ../log/cpu-*.log > ../log/cpu.log
rm ../log/cpu-*.log
rm cpu-*.log