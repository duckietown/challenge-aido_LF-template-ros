# syntax=docker/dockerfile:1.4

# parameters
ARG PROJECT_NAME="challenge-aido_LF-template-ros"
ARG PROJECT_DESCRIPTION="This template bridges Duckiematrix world I/O into the ente ROS lane-following stack."
ARG PROJECT_MAINTAINER="Liam Paull"
ARG PROJECT_ICON="cube"
ARG PROJECT_FORMAT_VERSION="1"

# ==================================================>
# ==> Do not change the code below this line
ARG ARCH=amd64
ARG DISTRO=ente
ARG DOCKER_REGISTRY=docker.io
ARG BASE_REPOSITORY=dt-core
ARG BASE_ORGANIZATION=duckietown
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG LAUNCHER=default

FROM ${DOCKER_REGISTRY}/${BASE_ORGANIZATION}/${BASE_REPOSITORY}:${BASE_TAG} AS base

ARG ARCH
ARG DISTRO
ARG DOCKER_REGISTRY
ARG PROJECT_NAME
ARG PROJECT_DESCRIPTION
ARG PROJECT_MAINTAINER
ARG PROJECT_ICON
ARG PROJECT_FORMAT_VERSION
ARG BASE_TAG
ARG BASE_REPOSITORY
ARG BASE_ORGANIZATION
ARG LAUNCHER
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH
ARG TARGETVARIANT
ARG PIP_INDEX_URL="https://pypi.org/simple"

RUN dt-args-check \
    "PROJECT_NAME" "${PROJECT_NAME}" \
    "PROJECT_DESCRIPTION" "${PROJECT_DESCRIPTION}" \
    "PROJECT_MAINTAINER" "${PROJECT_MAINTAINER}" \
    "PROJECT_ICON" "${PROJECT_ICON}" \
    "PROJECT_FORMAT_VERSION" "${PROJECT_FORMAT_VERSION}" \
    "ARCH" "${ARCH}" \
    "DISTRO" "${DISTRO}" \
    "DOCKER_REGISTRY" "${DOCKER_REGISTRY}" \
    "BASE_REPOSITORY" "${BASE_REPOSITORY}" \
    && dt-check-project-format "${PROJECT_FORMAT_VERSION}"

ARG PROJECT_PATH="${CATKIN_WS_DIR}/src/${PROJECT_NAME}"
ARG PROJECT_LAUNCHERS_PATH="${LAUNCHERS_DIR}/${PROJECT_NAME}"
RUN mkdir -p "${PROJECT_PATH}" "${PROJECT_LAUNCHERS_PATH}" /data/config
WORKDIR "${PROJECT_PATH}"

ENV DT_PROJECT_NAME="${PROJECT_NAME}" \
    DT_PROJECT_DESCRIPTION="${PROJECT_DESCRIPTION}" \
    DT_PROJECT_MAINTAINER="${PROJECT_MAINTAINER}" \
    DT_PROJECT_ICON="${PROJECT_ICON}" \
    DT_PROJECT_PATH="${PROJECT_PATH}" \
    DT_PROJECT_LAUNCHERS_PATH="${PROJECT_LAUNCHERS_PATH}" \
    DT_LAUNCHER="${LAUNCHER}" \
    PIP_INDEX_URL="${PIP_INDEX_URL}" \
    VEHICLE_NAME=agent \
    ROS_MASTER_URI=http://localhost:11311 \
    DISABLE_CONTRACTS=1

COPY ./dependencies.* "${PROJECT_PATH}/"
RUN dt-pip3-install "${PROJECT_PATH}/dependencies.*"

RUN git clone https://github.com/duckietown/duckiefleet.git /data/config

COPY ./assets/calibrations /tmp/runtime-calibrations
COPY ./scripts/install-runtime-calibrations.sh /usr/local/bin/install-runtime-calibrations
RUN chmod +x /usr/local/bin/install-runtime-calibrations && \
    /usr/local/bin/install-runtime-calibrations \
        /tmp/runtime-calibrations \
        /data/config/calibrations \
        map_0/vehicle_0

COPY ./solution/. "${PROJECT_PATH}/packages"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin build --workspace ${CATKIN_WS_DIR}/

COPY ./launchers/. "${PROJECT_LAUNCHERS_PATH}/"
RUN dt-install-launchers "${PROJECT_LAUNCHERS_PATH}"

CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

LABEL \
    org.duckietown.label.project.name="${PROJECT_NAME}" \
    org.duckietown.label.project.description="${PROJECT_DESCRIPTION}" \
    org.duckietown.label.project.maintainer="${PROJECT_MAINTAINER}" \
    org.duckietown.label.project.icon="${PROJECT_ICON}" \
    org.duckietown.label.project.path="${PROJECT_PATH}" \
    org.duckietown.label.project.launchers.path="${PROJECT_LAUNCHERS_PATH}" \
    org.duckietown.label.format.version="${PROJECT_FORMAT_VERSION}" \
    org.duckietown.label.platform.os="${TARGETOS}" \
    org.duckietown.label.platform.architecture="${TARGETARCH}" \
    org.duckietown.label.platform.variant="${TARGETVARIANT}" \
    org.duckietown.label.code.distro="${DISTRO}" \
    org.duckietown.label.code.launcher="${LAUNCHER}" \
    org.duckietown.label.code.python.registry="${PIP_INDEX_URL}" \
    org.duckietown.label.base.organization="${BASE_ORGANIZATION}" \
    org.duckietown.label.base.repository="${BASE_REPOSITORY}" \
    org.duckietown.label.base.tag="${BASE_TAG}"
