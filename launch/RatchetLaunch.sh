#!/bin/bash

#TODO: make sure roscore boots first, change perms so must be sudoed (?), make it pretty


tmux new-session -d -s RatchetStack
tmux send-keys 'roscore &'
tmux send-keys 'rosrun captain captain_runner.py' 'C-m'
tmux split-window -h
tmux send-keys 'rosrun tactics tactics_runner.py' 'C-m'
tmux split-window -v
tmux send-keys 'rosrun airmar airmar_runner.py' 'C-m'
tmux split-window -h
tmux send-keys 'rosrun servo_control servoControl.py' 'C-m'
tmux split-window -v
tmux send-keys 'rosrun sails_rudder sails_rudder_runner.py' 'C-m'
tmux split-window -h 
tmux -2 attach-session -t RatchetStack
