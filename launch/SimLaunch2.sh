#!/bin/bash

# Version without tmux

source devel/setup.sh
rosnode kill -a

rosrun rosbridge_server rosbridge_websocket &
rosrun captain sim_helper.py &
rosrun captain captain_runner.py &
rosrun tactics tactics_runner.py &
rosrun sails_rudder sails_rudder_runner.py &
rosrun speed_calculator speed_calculator_runner.py &
rosrun simulator sim_runner.py &
rosrun airmar airmar_smoother.py &

cd ../sailbot-dash
npm start