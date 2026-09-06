FROM osrf/ros:humble-desktop

ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# 1. Konfiguracja uzytkownika
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 2. Narzedzia, ROS 2, Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    nano vim git curl wget lsb-release gnupg python3-pip \
    ros-humble-vision-msgs \
    ros-humble-pcl-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-image-transport \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    rapidjson-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y gz-harmonic ros-humble-ros-gz libgz-sim8-dev \
    && rm -rf /var/lib/apt/lists/*

# Instalacja GeographicLib dla MAVROS
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

USER $USERNAME
WORKDIR /home/$USERNAME

# 3. Wtyczka ArduPilot-Gazebo
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git \
    && cd ardupilot_gazebo \
    && mkdir build && cd build \
    && export GZ_VERSION=harmonic \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc)

# 4. ArduPilot (wymuszenie wersji Copter-4.5)
RUN git clone https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git checkout Copter-4.5 \
    && git submodule update --init --recursive

RUN cd ardupilot \
    && USER=$USERNAME SKIP_APJ_TOOL=1 SKIP_AP_EXT_ENV=1 Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN pip3 install --user mavproxy pymavlink

RUN cd ardupilot \
    && ./modules/waf/waf-light configure --board sitl \
    && ./modules/waf/waf-light copter

# 5. Zmienne srodowiskowe
ENV GZ_VERSION=harmonic
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/$USERNAME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
ENV GZ_SIM_RESOURCE_PATH=/home/$USERNAME/ardupilot_gazebo/models:/home/$USERNAME/ardupilot_gazebo/worlds:/home/$USERNAME/ardupilot/Tools/simulation_install/gazebo/models:${GZ_SIM_RESOURCE_PATH}

RUN echo "export PATH=\$PATH:/home/$USERNAME/ardupilot/Tools/autotest" >> /home/$USERNAME/.bashrc \
    && echo "export PATH=\$PATH:/home/$USERNAME/.local/bin" >> /home/$USERNAME/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

WORKDIR /home/$USERNAME/ros2_ws