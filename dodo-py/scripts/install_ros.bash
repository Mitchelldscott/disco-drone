#! /bin/bash


#
#	Setup ROS repositories
#

echo -e "\n\tSetting up ROS ${ROS_DISTRO} ${ROS_PKG}\n"

$SUDO sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'


#
# Setup ROS keys
# 

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | $SUDO apt-key add -

$SUDO apt update


#
# Install ROS
#

echo -e "\n\tInstalling ros-${ROS_DISTRO}-${ROS_PKG}\n"

$SUDO apt install -y --no-install-recommends ros-${ROS_DISTRO}-${ROS_PKG} \
											ros-${ROS_DISTRO}-rqt \
											ros-${ROS_DISTRO}-tf2 \
											ros-${ROS_DISTRO}-rviz \
											ros-${ROS_DISTRO}-xacro \
											ros-${ROS_DISTRO}-catkin \
											ros-${ROS_DISTRO}-tf2-py \
											ros-${ROS_DISTRO}-rosbash \
											ros-${ROS_DISTRO}-roslaunch \
											ros-${ROS_DISTRO}-rqt-robot-plugins \
											ros-${ROS_DISTRO}-rqt-common-plugins \

#
# Install ROS dependencies
#

# echo -e "\n\tInstalling ros dependencies\n"

# pip3 install -r --no-input ${PROJECT_ROOT}/dodo-py/data/install/ros_requirements.txt


#
# Init rosdep
#

# if [[ "${HOSTNAME}" != "edge"* ]]; then
echo -e "\n\tSetting up rosdep\n"

$SUDO apt update
$SUDO apt install -y python3-rosdep2

cd /opt/ros/${ROS_DISTRO}

$SUDO rosdep init
rosdep update
# fi


#
#	Source buff.bash
#

# $SUDO apt update 

cd ${PROJECT_ROOT}

# echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# source ${PROJECT_ROOT}/dodo-py/dodo.bash

#
# Install cv2 bridge for python3
#

# echo -e "\n\tCloning vision_opencv to ${HOME}/opencv_ws\n"

# cd $HOME && mkdir opencv_ws && cd opencv_ws && \

# git clone -b noetic https://github.com/ros-perception/vision_opencv.git src/vision_opencv && \

# cd src/vision_opencv  && sed -i 's/python37/python3/g' cv_bridge/CMakeLists.txt && cd ../.. && catkin init && \

# catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/$(uname -m)-linux-gnu/libpython3.6m.so --install --extend /opt/ros/melodic && \

# catkin build cv_bridge && $SUDO cp -r install/lib/python3/dist-packages/* /opt/ros/melodic/lib/python2.7/dist-packages/ && cd $HOME

# if [[ -d ${HOME}/opencv_ws ]]; then
# 	rm -rf ${HOME}/opencv_ws
# fi

#
#	Return to project
# 
# cd ${PROJECT_ROOT}






