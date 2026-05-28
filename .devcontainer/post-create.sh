#!/bin/bash
set -e

# Ensure ROS setup is sourced in bashrc
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || \
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Ensure workspace overlay is sourced in bashrc
grep -qxF '[ -f /workspaces/sdk_deploy/install/setup.bash ] && source /workspaces/sdk_deploy/install/setup.bash' ~/.bashrc || \
  echo '[ -f /workspaces/sdk_deploy/install/setup.bash ] && source /workspaces/sdk_deploy/install/setup.bash' >> ~/.bashrc

# Create and setup simulation venv if it doesn't exist
if [ ! -d /workspaces/sdk_deploy/venv ]; then
  python3 -m venv /workspaces/sdk_deploy/venv
  /workspaces/sdk_deploy/venv/bin/pip install --upgrade pip wheel packaging build scikit-build-core
  /workspaces/sdk_deploy/venv/bin/pip install -r /workspaces/sdk_deploy/requirements.txt
  touch /workspaces/sdk_deploy/venv/COLCON_IGNORE
fi
