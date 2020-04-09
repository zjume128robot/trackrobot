#!/bin/zsh

cd /home/robot/install/share/localization/cfg/huake/
filesum=1
for file in `ls -r | grep "auto"`; do
    if [ $filesum -gt 10 ]; then
        rm -rf $file
    else
        filesum=`expr $filesum + 1`
    fi
done

