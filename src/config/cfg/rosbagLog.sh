#!/bin/bash

# this script is use for rosbag record data
cd ~
mkdir rosbagLog
cd rosbagLog
rosbag record /scan /tf /odom
bash
