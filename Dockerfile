FROM spgc/duckiebot-base-image:latest

SHELL ["/bin/bash", "-c"]

WORKDIR /ws

# Copy requirements files
COPY requirements-apt.txt .
COPY requirements-python.txt .

# Install apt packages (alsaaudio, alsa-utils, tmux)
RUN set -e; \
    if [ -s requirements-apt.txt ]; then \
        apt-get update; \
        xargs -a requirements-apt.txt apt-get install -y; \
        rm -rf /var/lib/apt/lists/*; \
    fi

# Install ALSA development libraries
RUN apt-get update && \
    apt-get install -y libasound2-dev && \
    rm -rf /var/lib/apt/lists/*

# Install python packages (numpy)
RUN set -e; \
    if [ -s requirements-python.txt ]; then \
        pip3 install -r requirements-python.txt; \
    fi

# Upgrade pyalsaaudio for Python 3.10+ compatibility
RUN pip3 install --upgrade pyalsaaudio

# Copy entire project (source code, scripts, etc.)
COPY . /ws/

# Make launch script executable
RUN chmod +x /ws/launch_robot.sh

# Build ROS2 workspace
RUN set -e && \
    source /opt/ros/humble/setup.bash && \
    colcon build && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ws/install/local_setup.bash" >> ~/.bashrc && \
    echo "cd /ws" >> ~/.bashrc

CMD ["bash", "-c", "while true; do sleep 3600; done"]
