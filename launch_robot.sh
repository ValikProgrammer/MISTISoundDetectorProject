#!/bin/bash
# Tmux launch script for Duckiebot sound hunter

SESSION="robot"

# Kill existing session if it exists
tmux has-session -t $SESSION 2>/dev/null
if [ $? -eq 0 ]; then
    tmux kill-session -t $SESSION
fi

# Set tmux options for better history
tmux set-option -g history-limit 50000

# Create new session and first window
tmux new-session -d -s $SESSION -n "control"

# Setup workspace in first pane
tmux send-keys -t $SESSION:control.0 "cd /ws" C-m
tmux send-keys -t $SESSION:control.0 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:control.0 "source install/local_setup.bash" C-m

# Split window vertically (left/right)
tmux split-window -h -t $SESSION:control

# Split left pane horizontally (top/bottom)
tmux select-pane -t $SESSION:control.0
tmux split-window -v -t $SESSION:control.0

# Split right pane horizontally (top/bottom)
tmux select-pane -t $SESSION:control.2
tmux split-window -v -t $SESSION:control.2

# Now we have 4 panes:
# 0: left top
# 1: left bottom
# 2: right top
# 3: right bottom
# (We'll split pane 2 later into 2 sub-panes)

# Setup each pane with workspace
for i in {0..3}; do
    tmux send-keys -t $SESSION:control.$i "cd /ws" C-m
    tmux send-keys -t $SESSION:control.$i "source /opt/ros/humble/setup.bash" C-m
    tmux send-keys -t $SESSION:control.$i "source install/local_setup.bash" C-m
done

# Left top: movement (auto-start)
tmux send-keys -t $SESSION:control.0 "ros2 run movement movement.py" C-m

# Left bottom: blinker (auto-start)
tmux send-keys -t $SESSION:control.1 "ros2 run blinker blinker.py" C-m

# Right top: split into two monitors (side-by-side)
# Split pane 2 horizontally (creates pane 3, shifts old pane 3 to pane 4)
tmux select-pane -t $SESSION:control.2
tmux split-window -h -t $SESSION:control.2

# Setup the new pane (pane 3)
tmux send-keys -t $SESSION:control.3 "cd /ws" C-m
tmux send-keys -t $SESSION:control.3 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:control.3 "source install/local_setup.bash" C-m

# Setup the shifted pane (pane 4)
tmux send-keys -t $SESSION:control.4 "cd /ws" C-m
tmux send-keys -t $SESSION:control.4 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION:control.4 "source install/local_setup.bash" C-m

# Now we have 5 panes:
# 0: left top (movement)
# 1: left bottom (blinker)
# 2: right top-left (frequency volume)
# 3: right top-right (total volume)
# 4: right bottom (joystick)

# Right top left: frequency volume monitor
tmux send-keys -t $SESSION:control.2 "ros2 topic echo /\$VEHICLE_NAME/frequency_volume_stream" C-m

# Right top right: total volume monitor
tmux send-keys -t $SESSION:control.3 "ros2 topic echo /\$VEHICLE_NAME/total_volume_stream" C-m

# Right bottom: joystick (ready, don't auto-start)
tmux send-keys -t $SESSION:control.4 "ros2 launch robot launch.xml"

# Create second window for audio processing
tmux new-window -t $SESSION -n "audio"

# Split vertically
tmux split-window -h -t $SESSION:audio

# Setup both panes
for i in {0..1}; do
    tmux send-keys -t $SESSION:audio.$i "cd /ws" C-m
    tmux send-keys -t $SESSION:audio.$i "source /opt/ros/humble/setup.bash" C-m
    tmux send-keys -t $SESSION:audio.$i "source install/local_setup.bash" C-m
done

# Left: audio capture (auto-start)
tmux send-keys -t $SESSION:audio.0 "ros2 run audio audio.py" C-m

# Right: volume processor (auto-start)
tmux send-keys -t $SESSION:audio.1 "ros2 run audio frequency_volume_processor --ros-args \
  -p target_frequency:=500.0 \
  -p frequency_bandwidth:=50.0 \
  -p log_interval:=1" C-m

# Select first window and attach
tmux select-window -t $SESSION:control
tmux select-pane -t $SESSION:control.0

# Attach to session
tmux attach-session -t $SESSION

