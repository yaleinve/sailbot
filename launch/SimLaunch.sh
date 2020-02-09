#!/bin/bash

tmux new-session -d -s SimStack "bash --rcfile launch/setup.bash"
tmux set-option -s -t SimStack default-command "bash --rcfile launch/setup.bash"
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-layout tile

tmux select-pane -t 0
tmux send-keys 'rosrun rosbridge_server rosbridge_websocket'  'C-m'

tmux select-pane -t 1
tmux send-keys 'rosrun captain sim_helper.py' 'C-m'

tmux select-pane -t 2
tmux send-keys 'rosrun captain captain_runner.py' 'C-m'

tmux select-pane -t 3
tmux send-keys 'rosrun tactics tactics_runner.py' 'C-m'

tmux new-window -a
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-layout tile

tmux select-pane -t 0
tmux send-keys 'rosrun sails_rudder sails_rudder_runner.py' 'C-m'

tmux select-pane -t 1
tmux send-keys 'rosrun speed_calculator speed_calculator_runner.py' 'C-m'

tmux select-pane -t 2
tmux send-keys 'rosrun simulator sim_runner.py' 'C-m'

tmux -2 attach-session -t SimStack
