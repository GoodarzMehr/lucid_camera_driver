# ARG ROS_DISTRO=noetic

# FROM ros:${ROS_DISTRO}-ros-core AS build-env
# ENV DEBIAN_FRONTEND=noninteractive \
#     BUILD_HOME=/var/lib/build \
#     LCD_SDK_PATH=/opt/lucid_camera_driver \
#     ARENA_ROOT=${LCD_SDK_PATH}}/ArenaSDK_Linux_x64 \
#     ARENA_CONFIG_ROOT=${LCD_SDK_PATH}/lucid_camera_driver

# RUN set -xue \
# # Kinetic and melodic have python3 packages but they seem to conflict
# && [ $ROS_DISTRO = "noetic" ] && PY=python3 || PY=python \
# # Turn off installing extra packages globally to slim down rosdep install
# && echo 'APT::Install-Recommends "0";' > /etc/apt/apt.conf.d/01norecommend \
# && apt-get update \
# && apt-get install -y \
#  build-essential cmake \
#  fakeroot dpkg-dev debhelper git \
#  $PY-rosdep $PY-rospkg $PY-bloom

# # Set up non-root build user
# ARG BUILD_UID=1000
# ARG BUILD_GID=${BUILD_UID}

# RUN set -xe \
# && groupadd -o -g ${BUILD_GID} build \
# && useradd -o -u ${BUILD_UID} -d ${BUILD_HOME} -rm -s /bin/bash -g build build

# # Install build dependencies using rosdep
# COPY --chown=build:build cav_msgs/package.xml ${LCD_SDK_PATH}/cav_msgs/package.xml
# COPY --chown=build:build dummy/package.xml ${LCD_SDK_PATH}/lucid_camera_driver/src/arena_camera/package.xml
# COPY --chown=build:build lucid_camera_driver/src/camera_control_msgs/package.xml ${LCD_SDK_PATH}/lucid_camera_driver/src/camera_control_msgs/package.xml

# RUN set -xe \
# && apt-get update \
# && rosdep init \
# && rosdep update --rosdistro=${ROS_DISTRO} \
# && rosdep install -y --from-paths ${LCD_SDK_PATH}

# RUN sudo git clone --depth 1 https://github.com/vishnubob/wait-for-it.git ~/.base-image/wait-for-it &&\
#     sudo mv ~/.base-image/wait-for-it/wait-for-it.sh /usr/bin

# RUN rm ${LCD_SDK_PATH}/lucid_camera_driver/src/arena_camera/package.xml

FROM usdotfhwastol/carma-lucid-camera-driver:base AS build-env

# Set up build environment
COPY --chown=build:build lucid_camera_driver ${LCD_SDK_PATH}/lucid_camera_driver
COPY --chown=build:build cav_msgs ${LCD_SDK_PATH}/cav_msgs
COPY --chown=build:build ArenaSDK_Linux_x64 ${LCD_SDK_PATH}/ArenaSDK_Linux_x64

RUN cp /opt/ros/noetic/include/sensor_msgs/image_encodings.h /opt/ros/noetic/include/sensor_msgs/image_encodings.h.bak
RUN chown build:build /opt/ros/noetic/include/sensor_msgs/image_encodings.h.bak
COPY --chown=build:build lucid_camera_driver/inc/image_encodings.h /opt/ros/noetic/include/sensor_msgs/image_encodings.h

RUN chown build:build ${LCD_SDK_PATH}/ArenaSDK_Linux_x64/Arena_SDK_Linux_x64.conf
RUN chmod g+x ${LCD_SDK_PATH}/ArenaSDK_Linux_x64/Arena_SDK_Linux_x64.conf
RUN cd ${LCD_SDK_PATH}/ArenaSDK_Linux_x64 && ls -l && ./Arena_SDK_Linux_x64.conf

USER build:build
WORKDIR ${BUILD_HOME}

RUN set -xe \
&& mkdir src \
&& ln -s ${LCD_SDK_PATH} ./src

FROM build-env

RUN /opt/ros/${ROS_DISTRO}/env.sh catkin_make -DCMAKE_BUILD_TYPE=Release 

# Command for running Lucid Vision Labs ROS:
CMD ["bash", "-c", "set -e \
&& . ./devel/setup.bash \
&& rosrun arena_camera arena_camera_node \
", "ros-entrypoint"]