FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME="ubuntu"
ARG UID_AND_GID=1234

# 基本ツール
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    locales \
    sudo \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-pytest \
    libgtest-dev \
    libpcl-dev \
    libopencv-dev

ENV USERNAME=$USERNAME
ENV USER=$USERNAME
RUN groupadd -g $UID_AND_GID $USERNAME && \
    useradd -ms /bin/bash -u $UID_AND_GID -g $UID_AND_GID -d /home/$USERNAME $USERNAME && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# ロケール設定
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS 2 GPG鍵とリポジトリ追加
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# ROS 2 Humbleとツールをインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-vcstool \
	python3-rosdep \
	python3-argcomplete && \
	rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
	rosdep init && \
	rosdep fix-permissions

# 作業ディレクトリと環境変数
ENV ROS_DISTRO=humble
USER $USERNAME
WORKDIR /home/$USERNAME

# bashrcにROS設定を追記
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

