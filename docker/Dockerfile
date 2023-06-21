FROM ros:noetic-ros-base
#FROM nvidia/cuda:11.5.1-cudnn8-runtime-ubuntu20.04

#ENV CUDNN_VERSION 7.5.0.56 CUDA_VERSION=11.5.1

#ARG ROS_VERSION=noetic
#ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.7.1
#FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

##############################################################################
# 1. install bootstrap dependencies
##############################################################################
#RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential git cmake curl wget unzip ninja-build ca-certificates \
        gnupg gnupg2 lsb-release openjdk-8-jdk\
        python3-dev python3-pip python3-numpy python3-matplotlib python3-empy python3-tk \
        libopencv-dev libopenblas-dev libopenmpi-dev openmpi-bin openmpi-common gfortran \
        libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev \
        libtbb2 libtbb-dev libpostproc-dev libeigen3-dev libglew-dev \
        libgtk2.0-dev pkg-config libhdf5-dev libprotobuf-dev protobuf-compiler libboost-all-dev libgflags-dev g++

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-rosdep python3-rosinstall-generator python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir /app

WORKDIR /app

##############################################################################
# 2. download/build the ROS source
##############################################################################
#RUN rosdep init && rosdep update
RUN rosdep update

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

RUN mkdir ros_catkin_ws && cd ros_catkin_ws && \
    rosinstall_generator ${ROS_PKG} vision_msgs --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall
RUN cd ros_catkin_ws && mkdir src && \
    vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src && \
    apt-get update
RUN cd ros_catkin_ws && rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} --skip-keys python3-pykdl -y
RUN cd ros_catkin_ws && ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS_ROOT} -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
    rm -rf /var/lib/apt/lists/*

#RUN apt-get update && \
#    apt-get install -q -y ros-${ROS_DISTRO}-cv-bridge && \
#    apt-get install -q -y ros-${ROS_DISTRO}-vision-opencv

##############################################################################
# 3. download/install required pip packages
##############################################################################
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir setuptools cmake cython wheel numpy && \
    python3 -m pip install --no-cache-dir rospkg PyYAML attrdict argparse
    

##############################################################################
# 4. download/build OpenCV
##############################################################################
ENV OPENCV_VERSION 4.5.0

#ENV OPENCV_VERSION 3.4.19

RUN mkdir -p opencv-build opencv-$OPENCV_VERSION/build
RUN curl -L https://github.com/opencv/opencv/archive/$OPENCV_VERSION.tar.gz | \
    tar xz && curl -L https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.tar.gz | tar xz

WORKDIR /app/opencv-$OPENCV_VERSION/build

RUN cmake -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-$OPENCV_VERSION/modules \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D WITH_PYTHON=ON \
    -D BUILD_opencv_gpu=OFF \
    -D WITH_CUDA=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D BUILD_opencv_python2=OFF \
    -D BUILD_opencv_python3=ON \
    -D PYTHON_DEFAULT_EXECUTABLE=python3 \
    # -D CUDA_NVCC_FLAGS="-D_FORCE_INLINES" \
    # -D CUDNN_VERSION="8.2" \
    # -D CUDA_ARCH_BIN="${CUDA_DEF}" \
    # -D CUDA_ARCH_PTX="" \
    # -D WITH_CUDA=ON -D WITH_CUBLAS=ON -D WITH_CUDNN=ON \
    # -D WITH_CUFFT=ON \
    # -D CUDA_FAST_MATH=ON \
    # -D OPENCV_DNN_CUDA=ON \
    # -D ENABLE_FAST_MATH=OFF \
    -D BUILD_opencv_highgui=ON \
    -D WITH_EIGEN=ON \
    -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
    -D WITH_TBB=OFF \
    -D WITH_1394=OFF \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_DOCS=OFF \
    ..
RUN make -j$(nproc) && make install



WORKDIR ${PROJECT_ROOT}

##############################################################################
# 5. INSTALL OPENCL Driver
##############################################################################

RUN apt-get update && \
     apt-get install -y software-properties-common wget  
 
#INSTALL opencl driver
RUN add-apt-repository ppa:graphics-drivers/ppa
RUN apt-get update && \
     apt-get install -y nvidia-driver-510 nvidia-cuda-toolkit clinfo 



##############################################################################
# 6. Install Helios Arena SDK and Arena Python API
##############################################################################
WORKDIR /app

RUN mkdir -p helios

ARG HELIOS_DIR=/app/helios

RUN mkdir -p ${HELIOS_DIR}/arena_python

ARG HELIOS_ARENA_PYTH_DIR=${HELIOS_DIR}/arena_python

COPY ./installer/ArenaSDK_v0.1.68_Linux_x64.tar.gz ${HELIOS_DIR}/ArenaSDK_v0.1.68_Linux_x64.tar.gz

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash"

RUN tar -xvf ${HELIOS_DIR}/ArenaSDK_v0.1.68_Linux_x64.tar.gz -C ${HELIOS_DIR}

WORKDIR ${HELIOS_DIR}/ArenaSDK_Linux_x64

RUN /bin/bash -c "sh Arena_SDK_Linux_x64.conf"

RUN /bin/bash -c "export ARENA_ROOT=${HELIOS_DIR}/ArenaSDK_Linux_x64"

COPY ./installer/arena_api-2.3.3-py3-none-any.zip ${HELIOS_DIR}/arena_api-2.3.3-py3-none-any.zip

RUN unzip ${HELIOS_DIR}/arena_api-2.3.3-py3-none-any.zip -d ${HELIOS_ARENA_PYTH_DIR}

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir ${HELIOS_ARENA_PYTH_DIR}/arena_api-2.3.3-py3-none-any.whl && \
    python3 -m pip install -r ${HELIOS_ARENA_PYTH_DIR}/examples/requirements_lin.txt



##############################################################################
# 7. create a ROS project
##############################################################################
ARG PROJECT_ROOT=/root
ARG DEVEL_ROOT=/app
ARG WS_NAME=mnrobot_helios

ARG PKG_HELIOS=helios_ws

ARG PKG_HELIOS_ros=helios

# WORKDIR ${PROJECT_ROOT}
# RUN mkdir -p ${PROJECT_ROOT}/${WS_NAME}/script
# RUN mkdir -p ${PROJECT_ROOT}/${WS_NAME}/src
RUN mkdir -p ${PROJECT_ROOT}/${WS_NAME}/src/${PKG_HELIOS_ros}/src

#RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; cd ${WS_NAME}/src; catkin_create_pkg ${PKG_NAME} mnrobot_msgs actionlib_msgs rospy std_msgs image_transport cv_bridge"
#RUN /bin/bash -c "cd ${WS_NAME}; rosdep install --from-paths src -i -y --rosdistro noetic"

WORKDIR ${DEVEL_ROOT}

# COPY ./script/entrypoint.sh ${WS_NAME}/script/entrypoint.sh
# COPY ./script/roscore.sh ${WS_NAME}/script/roscore.sh


RUN apt-get update && \
#    apt-get install -y ros-${ROS_DISTRO}-usb-cam && \
#    apt-get install -y ros-${ROS_DISTRO}-image-view \
#     apt-get install -y ros-${ROS_DISTRO}-image-transport && \
     apt-get install -y python3-catkin-tools python3-osrf-pycommon vim ros-${ROS_DISTRO}-desktop-full

RUN python3 -m pip install --no-cache-dir scikit-image

#RUN git clone -b develop https://github.com/ros-drivers/usb_cam.git ${WS_NAME}/src/${PKG_USB_CAM}
#RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; cd ${WS_NAME}; catkin_make"

#WORKDIR ${PROJECT_ROOT}

COPY ./helios_test.py ${PROJECT_ROOT}/helios_test.py

COPY ./installer/arena_camera_ros ${HELIOS_DIR}/${PKG_HELIOS}

#RUN git clone https://github.com/lucidvisionlabs/arena_camera_ros.git ${HELIOS_DIR}/${PKG_HELIOS}

#WORKDIR ${HELIOS_DIR}/${PKG_HELIOS}

#RUN git checkout tags/v1.5





WORKDIR ${DEVEL_ROOT}

#WORKDIR ${PROJECT_ROOT}
#RUN source /opt/ros/noetic/setup.bash
#RUN ln -s /${HELIOS_DIR}/${PKG_HELIOS}/catkin_ws/src/CMakeLists.txt /${PROJECT_ROOT}/${WS_NAME}/src/${PKG_HELIOS_ros}/src/CMakeLists.txt
#RUN chmod +x /${PROJECT_ROOT}/${WS_NAME}/src/${PKG_HELIOS}/CMakeLists.txt
RUN ln -s /${HELIOS_DIR}/${PKG_HELIOS}/catkin_ws/src/arena_camera /${PROJECT_ROOT}/${WS_NAME}/src/${PKG_HELIOS_ros}/src/arena_camera
RUN ln -s /${HELIOS_DIR}/${PKG_HELIOS}/catkin_ws/src/camera_control_msgs /${PROJECT_ROOT}/${WS_NAME}/src/${PKG_HELIOS_ros}/src/camera_control_msgs
RUN chmod +x /${PROJECT_ROOT}/${WS_NAME}/src/${PKG_HELIOS_ros}/src/arena_camera/*
RUN chmod +x /${PROJECT_ROOT}/${WS_NAME}/src/${PKG_HELIOS_ros}/src/arena_camera/scripts/*
RUN chmod +x /${PROJECT_ROOT}/${WS_NAME}/src/${PKG_HELIOS_ros}/src/camera_control_msgs/*

RUN ln -s /opt/ros/noetic/include/sensor_msgs/image_encodings.h /opt/ros/noetic/include/sensor_msgs/image_encodings.h.bak
RUN rm /opt/ros/noetic/include/sensor_msgs/image_encodings.h 
RUN ln -s /${HELIOS_DIR}/${PKG_HELIOS}/catkin_ws/inc/image_encodings.h /opt/ros/noetic/include/sensor_msgs/image_encodings.h




#apt-utils unzip tar curl xz-utils ocl-icd-libopencl1 opencl-headers nvidia-modprobe kmod

#RUN mkdir -p /etc/OpenCL/vendors && \
#    echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd
#ENV NVIDIA_VISIBLE_DEVICES all
#ENV NVIDIA_DRIVER_CAPABILITIES compute,utility




#RUN ln -s ${DEVEL_ROOT}/${WS_NAME}/src/${PKG_FOOD} /${PROJECT_ROOT}/${WS_NAME}/src/${PKG_FOOD}
#RUN ln -s ${DEVEL_ROOT}/${WS_NAME}/src/${PKG_MNROBOT_MSGS} /${PROJECT_ROOT}/${WS_NAME}/src/${PKG_MNROBOT_MSGS}
#RUN ln -s ${DEVEL_ROOT}/${WS_NAME}/script/entrypoint.sh /${PROJECT_ROOT}/${WS_NAME}/script
#RUN ln -s ${DEVEL_ROOT}/${WS_NAME}/script/FileCameraZividOne.zfc /${PROJECT_ROOT}/${WS_NAME}/script
#RUN ln -s ${DEVEL_ROOT}/${WS_NAME}/script/client.sh /${PROJECT_ROOT}/${WS_NAME}/script
#RUN ln -s ${DEVEL_ROOT}/${WS_NAME}/script/roscore.sh /${PROJECT_ROOT}/${WS_NAME}/script
#RUN ln -s ${DEVEL_ROOT}/${WS_NAME}/src/helios_test.py /${PROJECT_ROOT}/${WS_NAME}/helios_test.py
#RUN git clone -b noetic https://github.com/ros-perception/vision_opencv.git ${WS_NAME}/src/vision_opencv
#COPY ./config/cvbridge_CMakeLists.txt ${WS_NAME}/src/vision_opencv/cv_bridge/CMakeLists.txt
#RUN git clone -b develop https://github.com/ros-drivers/usb_cam.git ${WS_NAME}/src/${PKG_USB_CAM}

#ARG nvidia_binary_version="510.108.03"
#ARG nvidia_binary="NVIDIA-Linux-x86_64-${nvidia_binary_version}.run"
#RUN wget -q https://us.download.nvidia.com/XFree86/Linux-x86_64/${nvidia_binary_version}/${nvidia_binary} && \
#    chmod +x ${nvidia_binary} && \
#    ./${nvidia_binary} --accept-license --ui=none --no-kernel-module --no-questions && \
#    rm -rf ${nvidia_binary}




WORKDIR ${PROJECT_ROOT}/${WS_NAME}
# RUN /bin/bash -c "rosdep install --from-paths ./ -i -y --rosdistro noetic"
RUN /bin/bash -c "export ARENA_ROOT=${HELIOS_DIR}/ArenaSDK_Linux_x64; export ARENA_CONFIG_ROOT=/${PROJECT_ROOT}/${WS_NAME}/src/${PKG_HELIOS_ros}; source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build"


#RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; cd ${WS_NAME}; catkin_make -DCATKIN_WHITELIST_PACKAGES=""; source devel/setup.bash"

RUN /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash"
RUN /bin/bash -c ". ${PROJECT_ROOT}/${WS_NAME}/devel/setup.bash"
RUN ln -s /usr/bin/python3 /usr/bin/python
