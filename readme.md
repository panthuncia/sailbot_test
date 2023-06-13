# Setup steps:

install ros2 humble
install gazebo garden
build humble branch of ros_gz_bridge https://github.com/gazebosim/ros_gz/tree/humble

# in ~/gz_ws :
build https://github.com/srmainwaring/asv_wave_sim.git
build https://github.com/panthuncia/asv_sim.git
build https://github.com/panthuncia/rs750.git

# in ros2_ws :
build https://github.com/panthuncia/sailbot_test

# install packages:
sudo apt-get install '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev

# add everything after this to ~/.bashrc:

#Source ROS distro's setup.bash

source /opt/ros/humble/setup.bash

export GZ_VERSION=garden

#source ros_gz_bridge

source ~/ws/install/setup.bash

#add models to GZ path

export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:\~/gz_ws/install/asv_sim2/lib:\~/gz_ws/install/gz-waves1/lib

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\~/gz_ws/src/rs750/rs750_gazebo/models:\~/gz_ws/src/rs750/rs750_gazebo/worlds:\~/gz_ws/src/asv_wave_sim/gz-waves-models/models:\~/gz_ws/src/asv_wave_sim/gz-waves-models/world_models:\~/gz_ws/src/asv_wave_sim/gz-waves-models/worlds

#source gz_ws

source ~/gz_ws/install/setup.bash

#source ros2_ws

source ~/ros2_ws/install/setup.bash