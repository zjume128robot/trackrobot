#!/bin/bash
cp .bash_aliases  ~/
cp startup.sh ~/ 
chmod +x ~/startup.sh 
cp rosbagLog.sh ~/
chmod +x ~/rosbagLog.sh 
#cp ipconfig.sh ~/
#chmod +x ~/ipconfig.sh
chmod -R +x ../../node_manage/bash/
#generate core dump file use pid name
sudo sh -c "echo 1 > /proc/sys/kernel/core_uses_pid"
