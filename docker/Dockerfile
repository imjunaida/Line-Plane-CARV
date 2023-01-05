FROM ros:noetic-ros-base-focal



RUN apt update -y && sudo apt install tmux sudo curl git libcgal-dev libboost-all-dev  libblas-dev liblapack-dev liblapacke-dev  ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev  libjpeg-dev libpng-dev libtiff-dev libeigen3-dev libopenexr-dev libopengl-dev libgtk2.0-dev  pkg-config -y


RUN git clone https://github.com/FFmpeg/FFmpeg.git && cd FFmpeg && git checkout release/3.4 
RUN cd FFmpeg && ./configure --disable-x86asm  --enable-shared  && sudo make install -j$nproc

RUN git clone https://github.com/opencv/opencv.git && cd opencv && git checkout tags/4.0.0 && mkdir build && cd build &&  cmake .. && sudo make install -j$nproc
RUN sudo apt update -y && sudo apt install libgl1-mesa-dev libglew-dev  -y 
RUN git clone https://github.com/stevenlovegrove/Pangolin && cd Pangolin && git checkout tags/v0.5 && mkdir build && cd build && cmake .. && sudo make install -j$nproc

RUN git clone https://gitlab.com/libeigen/eigen.git 
RUN cd eigen && git checkout 3.1.0 && mkdir build && cd build && cmake .. && sudo make install -j$nproc


COPY ./Line-Plane-CARV /Line-Plane-CARV
RUN sudo apt upgrade -y 
RUN apt update -y && sudo apt install ros-noetic-tf ros-noetic-image-transport ros-noetic-cv-bridge -y 
RUN  cd opencv && git checkout tags/3.2.0 &&  cd build && rm -rf * && cmake .. -DCMAKE_CXX_FLAGS="-fpermissive" && sudo make install -j$nproc

RUN cd /Line-Plane-CARV && chmod +x build.sh && ./build.sh
RUN cd /Line-Plane-CARV && chmod +x build_ros.sh && /bin/bash -c '. /opt/ros/noetic/setup.bash; export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/Line-Plane-CARV/Examples/ROS; chmod +x build_ros.sh && ./build_ros.sh' &&     /bin/bash -c '.  Examples/ROS/ORB_SLAM2/build/devel/setup.bash'

