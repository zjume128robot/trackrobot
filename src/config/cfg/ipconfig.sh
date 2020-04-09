#!/bin/bash
echo 'robot' | sudo -S ifconfig eth0 192.168.1.10 netmask 255.255.255.0
echo 'robot' | sudo -S ifconfig eth1 192.168.1.107 netmask 255.255.255.0
