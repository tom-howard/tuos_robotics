FROM osrf/ros:humble-desktop-full

ENV ROS_VER=humble

# RUN yes | unminimize # for manpages

RUN echo "[INFO] Update Ubuntu" && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -o Dpkg::Options::="--force-confnew" -fuy \
    nano sudo wslu keychain rsync software-properties-common && \
    DEBIAN_FRONTEND=noninteractive apt-get dist-upgrade -o Dpkg::Options::="--force-confnew" -fuy && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo "[INFO] Set locale" && \
    apt-get update && \
    apt-get install locales && \
    locale-gen en_GB en_GB.UTF-8 && \
    update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8

RUN echo "[INFO] Update Git" && \
    apt-add-repository -y ppa:git-core/ppa && \
    apt-get update && \
    apt-get install -y git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo "[INFO] Install Various System Tools" && \
    apt-get update && \
    apt-get install -y \
    curl \
    less \
    eog \
    dos2unix \
    vim \
    tmux \
    tree \
    wget \
    python3-pip \
    python3-scipy \
    python3-pandas \
    python3-pydantic && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo "[INFO] Install Additional ROS tools" &&\
    apt-get update && \
    apt-get install -y \
    ros-dev-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-${ROS_VER}-gazebo-* \
    ros-${ROS_VER}-cartographer \
    ros-${ROS_VER}-cartographer-ros \
    ros-${ROS_VER}-navigation2 \
    ros-${ROS_VER}-nav2-bringup \
    ros-${ROS_VER}-turtlebot3 \
    ros-${ROS_VER}-turtlebot3-msgs \
    ros-${ROS_VER}-turtlebot3-simulations && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV WSL_DEFAULT_USER=student
ENV TUOS_DIR=/home/tuos

RUN mkdir -p ${TUOS_DIR}

RUN echo "[INFO] Add a Default User Profile" && \
    adduser --disabled-password --gecos '' ${WSL_DEFAULT_USER} && \
    addgroup ${WSL_DEFAULT_USER} sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' | tee -a /etc/sudoers && \
    chown -R ${WSL_DEFAULT_USER}:${WSL_DEFAULT_USER} ${TUOS_DIR} && \ 
    bash -c 'echo -e "[user]\ndefault=${WSL_DEFAULT_USER}"' | tee /etc/wsl.conf

WORKDIR ${TUOS_DIR}

RUN git clone https://github.com/tom-howard/tuos_robotics.git
WORKDIR ${TUOS_DIR}/tuos_robotics
RUN git checkout humble
WORKDIR ${TUOS_DIR}/tuos_robotics/wsl/source
RUN install waffle /usr/local/bin/ && \
    install wsl_ros /usr/local/bin/

USER ${WSL_DEFAULT_USER}

ENV WSL_HOME_DIR=/home/${WSL_DEFAULT_USER}
ENV SCRIPTS_PATH=${WSL_HOME_DIR}/.tuos
ENV SRC_PATH=${TUOS_DIR}/tuos_robotics/wsl/source

WORKDIR ${WSL_HOME_DIR}
RUN echo "[INFO] Configuring User Profile" && \
    mkdir ${SCRIPTS_PATH} && \
    cp ${SRC_PATH}/wsl_ros_setup.sh ${SCRIPTS_PATH} && \
    rm -f ${WSL_HOME_DIR}/.bash_aliases && \
    cp ${SRC_PATH}/bash_aliases ${WSL_HOME_DIR}/.bash_aliases && \
    bash -c 'echo -e "\nsource '${SCRIPTS_PATH}'/wsl_ros_setup.sh\n"' | tee -a ${WSL_HOME_DIR}/.bashrc && \
    sudo -u ${WSL_DEFAULT_USER} -H git config --global init.defaultBranch main && \
    sudo -u ${WSL_DEFAULT_USER} -H git config --global push.default simple

RUN echo "[INFO] Creating a ROS2 Workspace" && \
    mkdir -p ${WSL_HOME_DIR}/ros2_ws/src/

WORKDIR ${WSL_HOME_DIR}/ros2_ws/
RUN cd src && \
    git clone https://github.com/tom-howard/tuos_ros.git && \
    cd tuos_ros && \
    git checkout humble

RUN echo "[INFO] Creating a Path for a U drive (if available)" && \
    sudo mkdir -p /mnt/u

ENV COLCON_PREFIX_PATH=/home/${WSL_DEFAULT_USER}/ros2_ws/src

ARG VERSION
ARG XSERVER

# this bit not working...
RUN echo "[INFO] Store ARGs to file" && \
    echo "${VERSION}" > ${TUOS_DIR}/wsl_ros_ver && \
    echo "export XSERVER=${XSERVER}" > ${SCRIPTS_PATH}/xserver.sh

CMD ["bash", "-l"]