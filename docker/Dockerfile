FROM moveit/moveit2:jazzy-release

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/tmpfs/reach_ros
ARG INSTALL_DIR=/opt/reach_ros
RUN apt update -y -qq \
    && rosdep update
  
# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=tmpfs,target=${WORKSPACE_DIR} --mount=type=bind,target=${WORKSPACE_DIR}/src/reach_ros2 \
  cd ${WORKSPACE_DIR} \ 
  && source /opt/ros/jazzy/setup.bash \
  && vcs import ${WORKSPACE_DIR}/src < ${WORKSPACE_DIR}/src/reach_ros2/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
  && colcon build --cmake-args \
  && cp -r ${WORKSPACE_DIR}/install ${INSTALL_DIR}