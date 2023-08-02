import os
import SET

session_name = 'ASSP'

# os.system("tmux kill-session -t " + session_name)
# os.system('tmux kill-server')

os.system("tmux new -d -s "+session_name+" -x 500 -y 500")

agent_list=SET.agent_list
Num=len(agent_list)

for i in range(Num):
    os.system('tmux split-window ; tmux select-layout tiled')

for i in range(Num):

    # os.system('tmux send-keys -t '+session_name+':0.'+str(i) +' "conda activate $conda-env$" C-m')

    os.system('tmux send-keys -t '+session_name+':0.'+str(i) +' "source devel/setup.bash" C-m')

    index=agent_list[i]['index']
    os.system('tmux send-keys -t '+session_name+':0.'+str(i) +' "python3 src/planner/scripts/Planner.py '+str(index)+'" '+' C-m')

os.system('tmux attach -t '+session_name)