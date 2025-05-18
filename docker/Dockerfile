FROM ubuntu:16.04
ENV DEBIAN_FRONTEND noninteractive
ENV LC_ALL C.UTF-8
ENV LANG C.UTF-8

# create docker user
RUN apt-get update \
 && apt-get install --no-install-recommends -y sudo patch \
 && useradd -ms /bin/bash docker \
 && echo 'docker ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
 && adduser docker sudo \
 && apt-get clean \
 && mkdir -p /home/docker \
 && sudo chown -R docker /home/docker \
 && sudo chown -R docker /usr/local/bin \
 && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
USER docker

# install dependencies
RUN sudo apt-get update \
 && sudo apt-get install -y --no-install-recommends \
      gcc \
      g++ \
      gcovr \
      libc6-dev \
      bc \
      gdb \
      python3 \
      python3-pip \
      python3-setuptools \
 && sudo apt-get clean \
 && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# upgrade pip to a compatible version and install numpy
RUN python3 -m pip install --user --upgrade pip==20.3.4 \
 && python3 -m pip install --user numpy

WORKDIR /experiment
COPY test_driver.cpp /experiment/lib/
COPY controller.c /experiment/source/
COPY controller.h /experiment/
# COPY controller_sanity.c /experiment/source/
COPY test.sh /experiment
COPY check_distance.py /experiment
COPY test /experiment/test
RUN sudo chown -R docker /experiment
RUN sudo chown -R docker /tmp