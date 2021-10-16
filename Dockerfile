# Definition of Submission container

ARG ARCH=amd64
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}

ARG DOCKER_REGISTRY=docker.io
FROM ${DOCKER_REGISTRY}/duckietown/dt-ros-commons:${BASE_TAG}
WORKDIR /code


# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt

ENV DEBIAN_FRONTEND=noninteractive

# DO NOT MODIFY: your submission won't run if you do
RUN apt-get update -y && \
    apt-get install -y apt-utils && \
    apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python3-tk \
         python3-wheel \
         python3-pip  \
         libcairo2-dev \
         libjpeg-dev\
          libgif-dev\
         software-properties-common && \
     rm -rf /var/lib/apt/lists/*


# RUN apt-get update -y && \
#   add-apt-repository ppa:deadsnakes/ppa -y && \
#   apt-get update -y && \
#   apt-get install -y python3.7-dev && \
#   ln -sf /usr/bin/python3.7 /usr/bin/python3


RUN mkdir -p /data/config
# TODO this is just for the default.yamls - these should really be taken from init_sd_card
RUN git clone https://github.com/duckietown/duckiefleet.git /data/config

ARG PIP_INDEX_URL="https://pypi.org/simple"

RUN echo we have PIP_INDEX_URL=${PIP_INDEX_URL} $PIP_INDEX_URL $DOCKER_REGISTRY


ENV PIP_INDEX_URL=${PIP_INDEX_URL}

RUN echo we have PIP_INDEX_URL=${PIP_INDEX_URL} $PIP_INDEX_URL
RUN env


#RUN python3 -m pip check # XXX: fails
RUN python3 -m pip list

COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN python3 -m pip install --no-cache-dir -r .requirements.txt
RUN python3 -m pip check
RUN python3 -m pip list


# For ROS Agent - Need to upgrade Pillow for Old ROS stack
#RUN python3 -m pip install pillow --user --upgrade

RUN mkdir submission_ws

COPY submission_ws/src submission_ws/src
COPY launchers ./

# FIXME: what is this for? envs are not persisted
RUN /bin/bash -c "export PYTHONPATH="/usr/local/lib/python3.7/dist-packages:$PYTHONPATH""

ENV HOSTNAME=agent
ENV VEHICLE_NAME=agent
ENV ROS_MASTER_URI=http://localhost:11311

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . ${CATKIN_WS_DIR}/devel/setup.bash  && \
    catkin build --workspace /code/submission_ws

ENV DISABLE_CONTRACTS=1
CMD ["bash", "run_and_start.sh"]
