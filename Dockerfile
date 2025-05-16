FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# 必要な開発ツールのインストール
RUN apt-get update && apt-get install -y \
    python3-pip \
    clang \
    clang-format \
    curl \
    lsb-release \
    sudo \
    git \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# rosdep の初期化
RUN rosdep update

# リポジトリのクローン
WORKDIR /home/nitic-robot-club

RUN git clone https://github.com/NITIC-Robot-Club/nhk2025b_ros2.git
WORKDIR /home/nitic-robot-club/nhk2025b_ros2

RUN rosdep install -y --from-paths src --ignore-src && \
    colcon build --symlink-install
