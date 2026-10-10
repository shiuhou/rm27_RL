FROM ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake libeigen3-dev libopencv-dev libboost-all-dev \
    libceres-dev ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-tf ros-noetic-pcl-ros ros-noetic-tf2-eigen \
    python3-numpy python3-pytest time && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
