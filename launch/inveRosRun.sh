#!/bin/bash

clear
tmux new-session -d -s inve
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-layout tile

tmux select-pane -t 0
tmux send-keys 'rostopic echo /airmar_data' 'C-m'

tmux select-pane -t 1
tmux send-keys 'rostopic echo /competion_info' 'C-m'

tmux select-pane -t 2
tmux send-keys 'rostopic echo /leg_info' 'C-m'

tmux select-pane -t 3
tmux send-keys 'rostopic echo fix/latitude' 'C-m'

tmux select-pane -t 4
tmux send-keys 'rostopic echo fix/longitude' 'C-m'

tmux select-pane -t 5
tmux send-keys 'rostopic echo /sails_rudder_pos' 'C-m'

tmux select-pane -t 6
tmux send-keys 'rostopic echo /nav_targets' 'C-m'


tmux -2 attach-session -t inve


















