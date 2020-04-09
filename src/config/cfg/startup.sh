#!/bin/bash
#gnome-terminal -x zsh -c "source ~/.zshrc.local;roslaunch  node_manage #node_manage.launch;exec zsh"
gnome-terminal -x bash -c "source ~/.bashrc.local;roslaunch node_manage node_manage.launch;exec bash"
