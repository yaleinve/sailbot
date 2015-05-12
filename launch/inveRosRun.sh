#!/bin/sh

clear
spin[0]="-"
spin[1]="\\"
spin[2]="|"
spin[3]="/"
end=$((SECONDS+6))

cat << "EOF" 
          _______  _        _______   _________ _                 _______    _______  _______ _________ _        ______   _______ _________
|\     /|(  ___  )( \      (  ____ \  \__   __/( (    /||\     /|(  ____ \  (  ____ \(  ___  )\__   __/( \      (  ___ \ (  ___  )\__   __/
( \   / )| (   ) || (      | (    \/     ) (   |  \  ( || )   ( || (    \/  | (    \/| (   ) |   ) (   | (      | (   ) )| (   ) |   ) (   
 \ (_) / | (___) || |      | (__         | |   |   \ | || |   | || (__      | (_____ | (___) |   | |   | |      | (__/ / | |   | |   | |   
  \   /  |  ___  || |      |  __)        | |   | (\ \) |( (   ) )|  __)     (_____  )|  ___  |   | |   | |      |  __ (  | |   | |   | |   
   ) (   | (   ) || |      | (           | |   | | \   | \ \_/ / | (              ) || (   ) |   | |   | |      | (  \ \ | |   | |   | |   
   | |   | )   ( || (____/\| (____/\  ___) (___| )  \  |  \   /  | (____/\  /\____) || )   ( |___) (___| (____/\| )___) )| (___) |   | |   
   \_/   |/     \|(_______/(_______/  \_______/|/    )_)   \_/   (_______/  \_______)|/     \|\_______/(_______/|/ \___/ (_______)   )_(   
                                                                                                                                           
 ______   _______  _______           ______   _______  _______  _______  ______                __       _______  _______                   
(  __  \ (  ___  )(  ____ \|\     /|(  ___ \ (  ___  )(  ___  )(  ____ )(  __  \     |\     /|/  \     (  __   )(  __   )                  
| (  \  )| (   ) || (    \/| )   ( || (   ) )| (   ) || (   ) || (    )|| (  \  )    | )   ( |\/) )    | (  )  || (  )  |                  
| |   ) || (___) || (_____ | (___) || (__/ / | |   | || (___) || (____)|| |   ) |    | |   | |  | |    | | /   || | /   |                  
| |   | ||  ___  |(_____  )|  ___  ||  __ (  | |   | ||  ___  ||     __)| |   | |    ( (   ) )  | |    | (/ /) || (/ /) |                  
| |   ) || (   ) |      ) || (   ) || (  \ \ | |   | || (   ) || (\ (   | |   ) |     \ \_/ /   | |    |   / | ||   / | |                  
| (__/  )| )   ( |/\____) || )   ( || )___) )| (___) || )   ( || ) \ \__| (__/  )      \   /  __) (_ _ |  (__) ||  (__) |                  
(______/ |/     \|\_______)|/     \||/ \___/ (_______)|/     \||/   \__/(______/        \_/   \____/(_)(_______)(_______)                  
                                                                                                                                           

EOF

echo -n "[Booting up the dashboard...please stand by] ${spin[0]}"
while [ $SECONDS -lt $end ]
do
  for i in "${spin[@]}"
  do
        echo -ne "\b$i"
        sleep 0.1
  done
done

clear
tmux new-session -d -s inve
tmux send-keys 'rostopic echo /airmar_data' 'C-m'
tmux rename-window 'InVe'
tmux split-window -v
tmux send-keys 'rostopic echo /speed_stats' 'C-m'
tmux split-window -h
tmux send-keys 'rostopic echo /target_course' 'C-m'
tmux split-window -v
tmux send-keys 'rostopic echo /target_heading' 'C-m'
tmux select-pane -U
tmux select-pane -U
tmux split-window -h
tmux send-keys 'rostopic echo /leg_info' 'C-m'
tmux split-window -v
tmux send-keys 'rostopic echo fix/latitude' 'C-m'
tmux split-window -h
tmux send-keys 'rostopic echo fix/longitude' 'C-m'
tmux -2 attach-session -t inve


















