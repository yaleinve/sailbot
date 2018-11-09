#!/bin/bash

tmux new-session -d -s AllStack
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux select-layout tile

tmux select-pane -t 0
tmux send-keys 'rosrun captain hardware_helper.py' 'C-m'

tmux select-pane -t 1
tmux send-keys 'rosrun captain captain_runner.py' 'C-m'

tmux select-pane -t 2
tmux send-keys 'rosrun tactics tactics_runner.py' 'C-m'

tmux select-pane -t 3
tmux send-keys 'rosrun airmar airmar_runner.py'  'C-m'

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
tmux send-keys 'rosrun servo_control servoControl.py' 'C-m'

tmux new-window -a
tmux split-window -v
tmux select-layout tile

tmux -2 attach-session -t AllStack
