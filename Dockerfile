# Definition of Submission container

ARG ARCH=amd64
ARG MAJOR=daffy
ARG BASE_TAG=${MAJOR}-${ARCH}

FROM duckietown/dt-ros-commons:${BASE_TAG}
WORKDIR /code


# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt

# DO NOT MODIFY: your submission won't run if you do
RUN apt-get update -y && apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python3-tk \
         python3-wheel \
         python3-pip  \
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


ARG PIP_INDEX_URL
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN echo PIP_INDEX_URL=${PIP_INDEX_URL}

# Before installing
RUN echo PYTHONPATH=$PYTHONPATH
RUN pip3 install -U "pip>=20.2" pipdeptree

RUN pipdeptree
RUN pip list

# FIXME ros-commons is broken
RUN apt-get update && apt-get install -y libcairo2-dev libjpeg-dev libgif-dev

RUN pip3 install pycairo==1.19.1
RUN pip check

COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN  pip3 install --use-feature=2020-resolver -r .requirements.txt

RUN echo PYTHONPATH=$PYTHONPATH
RUN pipdeptree
RUN pip list

# For ROS Agent - Need to upgrade Pillow for Old ROS stack
#RUN pip3 install pillow --user --upgrade

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
