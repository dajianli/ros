#!/bin/bash

linear=0.0
angular=0.0

if [ $# -ge 1 ]; then
linear=$1
fi

if [ $# -ge 2 ]; then
angular=$2
fi

rostopic pub -1 /cmd_vel geometry_msgs/Twist  '{linear: {x: '$linear' , y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: '$angular' }}' 

