system_id=5

PID=$(pgrep -f system${system_id}_group0_trainer) &amp;&amp; echo 'neucyber' | sudo -S kill -9 $PID
PID=$(pgrep -f system${system_id}_group0_state0_predictor) &amp;&amp; echo 'neucyber' | sudo -S kill -9 $PID
PID=$(pgrep -f psychopy_presenter) &amp;&amp; echo 'neucyber' | sudo -S kill -9 $PID
PID=$(pgrep -f system${system_id}_group0_integrator) &amp;&amp; echo 'neucyber' | sudo -S kill -9 $PID

fastdds shm clean
# cd /share/ECoG_neural_group_classification/ &amp;&amp; python3 ros2_gui.py