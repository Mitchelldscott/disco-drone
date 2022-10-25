#! /bin/bash


#
# 	First install system dependencies through apt
#

echo -e "\n\tInstalling BuffCode Dependencies...\n"

$SUDO xargs apt install -y --no-install-recommends <${PROJECT_ROOT}/dodo-py/data/install/dependencies.txt



#
#	Install python requirements with pip3
#

echo -e "\n\tInstalling BuffCode python3 requirements\n"

pip3 install -r ${PROJECT_ROOT}/dodo-py/data/install/requirements.txt

