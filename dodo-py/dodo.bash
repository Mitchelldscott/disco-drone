#! /usr/bin/env bash


export PROJECT_ROOT=${PWD}
export HOSTNAME=$HOSTNAME 
export SUDO='sudo'
export ROBOT_IP=192.168.1.106

if [[ "${UBUNTU_VERSION}" == "20.04" ]]; then
	export ROS_DISTRO=noetic				# ROS for Ubuntu18
elif [[ "${UBUNTU_VERSION}" == "18.04" ]]; then
	export ROS_DISTRO=melodic
fi

#		Setup python tools

if [[ "${PATH}" != *"${PROJECT_ROOT}/dodo-py/bin"* ]]; then
	export PATH="${PROJECT_ROOT}/dodo-py/bin:${PATH}"
fi  

# If ROS is installed source the setup file

if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
	source /opt/ros/${ROS_DISTRO}/setup.bash
fi

# Only export if if not already in path

if [[ "${UBUNTU_VERSION}" == "18.04" ]]; then
	if [[ "${PYTHONPATH}" != *"${PROJECT_ROOT}/dodo-py/lib:"* ]]; then	
		export PYTHONPATH="${PROJECT_ROOT}/dodo-py/lib/python3/dist-packages:${PYTHONPATH}" 
	fi
fi 

# set ROS package path to buff-code so it can see buffpy

if [[ "${ROS_PACKAGE_PATH}" != *"disco-drone"* ]]; then
	export ROS_PACKAGE_PATH="${PROJECT_ROOT}:${ROS_PACKAGE_PATH}"
fi


alias install-py="cp ${PROJECT_ROOT}/src/serial_host/scripts/* ${PROJECT_ROOT}/dodo-py/lib && \
					cp ${PROJECT_ROOT}/src/localization/scripts/* ${PROJECT_ROOT}/dodo-py/lib && \
					cp ${PROJECT_ROOT}/src/locomotion/scripts/* ${PROJECT_ROOT}/dodo-py/lib && \
					chmod +x ${PROJECT_ROOT}/dodo-py/lib/*"

alias build-fw="cd ${PROJECT_ROOT}/src/firmware && \
					pio run && \
					cp .pio/build/teensy41/firmware.hex ${PROJECT_ROOT}/dodo-py/bin && \
					cd ${PROJECT_ROOT}"

alias upload-fw="cd ${PROJECT_ROOT}/src/firmware && \
					pio run -t upload && \
					cd ${PROJECT_ROOT}"

alias clean-dodo="rm -rf ${PROJECT_ROOT}/dodo-py/lib/*.py && \
					rm -rf ${PROJECT_ROOT}/dodo-py/bin/firmware.hex"

alias clean-fw="cd ${PROJECT_ROOT}/src/firmware && \
					pio run -t clean && \
					cd ${PROJECT_ROOT}"

alias deploy="build-fw && install-py && scp -r ${PROJECT_ROOT}/dodo-py ubuntu@${ROBOT_IP}:~/disco-drone"

alias sshbot="ssh ubuntu@${ROBOT_IP}"

alias dodo="cd ${PROJECT_ROOT}"
alias fw="cd ${PROJECT_ROOT}/src/firmware"