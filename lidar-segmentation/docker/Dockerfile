# start from ubuntu 16 base image
FROM ubuntu:16.04


ENV http_proxy=http://172.17.0.1:3128
ENV https_proxy=http://172.17.0.1:3128
ENV NO_PROXY=*.bosch.com,127.0.0.1


# Install some basic utilities
RUN apt-get update && apt-get install --no-install-recommends -y \
    curl ca-certificates \
    sudo git bzip2 libjpeg-dev libpng-dev \
    cmake build-essential libx11-6 libgtk2.0-0 \
    libcanberra-gtk-module protobuf-compiler libprotoc-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo "proxy_servers:" >> $HOME/.condarc && \
    echo "    https: $https_proxy" >> $HOME/.condarc && \
    echo "    http: $http_proxy" >> $HOME/.condarc && \
    echo "ssl_verify: False" >> $HOME/.condarc


# Copy pubkey for ros
COPY ros_pubkey /tmp/ros_pubkey
# add pubkey for ros
RUN apt-key add /tmp/ros_pubkey


# Install some more basics
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y \
    wget unzip  libbz2-dev qt5-default qtdeclarative5-dev libqt5x11extras5-dev \
    libomp-dev ccache libopencv-stitching-dev && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/*


# Set up sources list
RUN apt update && apt install -y lsb-release 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


# Install all the ROS shit
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full ros-kinetic-audio-common ros-kinetic-joystick-drivers python-wstool python-catkin-tools ros-kinetic-rosbash libtclap-dev \
    plantuml libpcap-dev libftgl-dev libgsl-dev python-pip rubber libmysqlcppconn-dev python-vcstool \
    ccache coinor-* libzmq-jni libzmq3-dev libzmq5 libzmqpp-dev libzmqpp3 libsvm-dev ros-kinetic-libg2o \
    wget python-dev python3 libpython-dev \
    python-matplotlib libeigen3-dev libfreeimage-dev libtclap-dev python-scipy python-h5py \
    libgeographic-dev libpcl-dev python-lxml libx264-dev python-yaml libbz2-dev libgsl-dev python-gtk2 \
    texlive-latex-extra libyaml-cpp-dev python-shapely google-mock libboost-all-dev libqt5core5a libproj-dev \
    libqt5widgets5 libxml++2.6-2v5 libxml++2.6-dev imagemagick libqglviewer-dev-qt4 python-dev python-numpy \
    python-rospkg libopencv-dev libtinyxml-dev libconsole-bridge-dev libblas-dev graphviz libqt5opengl5 \
    libpcl1.7 libsuitesparse-dev doxygen python-psutil \
    libbullet-dev libassimp-dev libjpeg-dev libogre-1.9-dev python-imaging libqt5gui5 lcov pkg-config \
    nano sudo linux-image-`uname -r`  linux-headers-`uname -r`


# Initialize rosdep
RUN rosdep init && rosdep update


# Environment setup
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc 
RUN /bin/bash -c "source ~/.bashrc"


#RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
#RUN echo "source /workspace/utils/setup.sh" >> ~/.bashrc


# Dependencies for building packages (this is a 100% unnecessary)
RUN apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential



# Install BOOST
ENV CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:/usr/include/python2.7/"
RUN cd /tmp && wget git.bp.emea.bosch.com:81/boost_1_58.zip && unzip boost_1_58.zip && \ 
    cp -r /tmp/boost_1_58_0/boost/ /opt/ros/kinetic/include/ && \
    cp /tmp/boost_1_58_0/stage/lib/* /opt/ros/kinetic/lib/ && \
    cd /tmp/boost_1_58_0 && ./b2 install
RUN rm -rf /tmp/boost*


# Add development tools and GUI
RUN apt update && apt install -y tmux zsh curl wget vim emacs24 libgl1-mesa-glx libgl1-mesa-dri mesa-utils unzip


# Install CATKIN-TOOLS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y python-catkin-tools


# Get PCL
RUN git clone https://github.com/PointCloudLibrary/pcl.git ~/pcl

# Install PCL
RUN mkdir ~/pcl/release
RUN cd ~/pcl/release && cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \
           -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON \
           -DCMAKE_INSTALL_PREFIX=/usr ~/pcl
RUN cd ~/pcl/release && make
RUN cd ~/pcl/release && make install



# When everything is finished, fck dat proxy shit up
#ENV http_proxy=""
#ENV https_proxy=""
#ENV NO_PROXY=""

