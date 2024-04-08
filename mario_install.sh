#!/usr/bin/env bash

set -e

red=`tput setaf 1`
green=`tput setaf 2`
reset=`tput sgr0`

echo "Installing ESP IDF"

_shell_="${SHELL#${SHELL%/*}/}"
# Check whether esp-idf has already been installed
if [ -d $HOME/esp/esp-idf ]; then
    echo "You already have installed esp-idf!"
else
    # System Detection and ESP-IDF Installation
    unameOut="$(uname -s)"
    case "${unameOut}" in
        Linux*)
            sudo apt update && sudo apt upgrade -y
            sudo usermod -a -G dialout $USER
            sudo apt install git wget flex bison gperf python3 python3-pip python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 -y
            sudo apt install python3-venv
            ;;
        Darwin*)
            if brew --version | grep -q 'Homebrew'; then
                echo "Homebrew is already installed"
            else 
                echo "installing homebrew"
                /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
                echo "homebrew installed successfully"
            fi
            brew install git cmake ninja dfu-util python3 pip 
            ;;
        *)          
            echo "Not supported: ${unameOut}"
            exit 1
            ;;
    esac

    # Create ESP Directory
    mkdir -p "$HOME/esp"
    pushd "$HOME"/esp || (echo "Error: Cannot Make Directory" && exit 1)

    # Clone ESP-IDF Repository
    git clone -b release/v5.1 --recursive https://github.com/espressif/esp-idf.git
    cd $HOME/esp/esp-idf
    ./install.sh esp32

    # Check if installation is successful
    . $HOME/esp/esp-idf/export.sh 
    idf.py --version | (grep "v5.1.2" && echo "Installation successful! Please restart your computer for the changes to take effect.") \
        || (echo "Installation failed" && exit 1) 
    # Set IDF Alias
    echo "alias get_idf='. $HOME/esp/esp-idf/export.sh'" >> $HOME/."$_shell_"rc
fi

# Clone Mario repository
if [ ! -d "$HOME/MARIO" ]; then
    cd "$HOME" || (echo "Error: Could not navigate to Home" && exit 1)
    echo "Cloning Mario"
    git clone --recursive https://github.com/AryanNanda17/MARIO.git /tmp/ros2_ws
else 
    echo "You already have Cloned MARIO!"
fi

unameOut="$(uname -s)"

# ROS2 Installation
case "${unameOut}" in
    Linux*)
        echo "Installing ROS 2 for Linux"
        
        # Check for UTF-8 locale
        locale
        
        sudo apt update && sudo apt install locales
        sudo locale-gen en_US en_US.UTF-8
        sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8
        
        locale  # verify settings
        
        sudo apt install software-properties-common
        sudo add-apt-repository universe
        
        sudo apt update && sudo apt install curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        
        sudo apt update
        sudo apt upgrade
        sudo apt install ros-humble-desktop
        echo "source /opt/ros/humble/setup.bash" >> $HOME/."$_shell_"rc

        # Install additional important ros2 packages 
        echo "Installing Additional Ros2 packages"
        sudo apt install ros-humble-desktop-full ros-humble-control* ros-humble-gazebo-ros2-control ros-humble-joint-state-* ros-humble-forward-command-controller ros-humble-robot-state-publisher ros-humble-robot-controllers*
        ;;
    Darwin*)
        echo "Installing miniconda"
        # Install Miniconda
        cd ~
        curl https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-arm64.sh -o Miniconda3-latest-MacOSX-arm64.sh
        chmod +x Miniconda3-latest-MacOSX-arm64.sh
        ./Miniconda3-latest-MacOSX-arm64.sh
        conda config --set auto_activate_base false

        echo "Installing ROS 2 for macOS"
        # Installing ROS 2 environment
        conda create --name ros_env
        conda activate ros_env
        conda config --env --add channels conda-forge
        conda config --env --add channels robostack-staging
        conda config --env --remove channels defaults
        conda install ros-humble-desktop
        conda deactivate
        conda activate ros_env

        # Install additional important ros2 packages
        echo "Installing Additional Ros2 packages"
        conda install ros-humble-desktop-full ros-humble-control-* ros-humble-gazebo-ros2-control ros-humble-joint-state-* ros-humble-forward-command-controller ros-humble-robot-state-publisher 

        ;;
    *)
        echo "ROS 2 installation is not supported on this operating system."
        exit 1
        ;;
esac

# Creating a ros2_ws
case "${unameOut}" in
    Linux*)
        echo "Creating a ros2_ws"
        sudo apt update 
        sudo apt install python3-colcon-common-extensions rosdep
        echo "source  /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> $HOME/."$_shell_"rc
        mkdir ~/ros2_ws
        cd ~/ros2_ws
        mkdir src 
        colcon build
        echo "source  ~/ros2_ws/install/setup.bash" >> $HOME/."$_shell_"rc
        ;;
    Darwin*)
        echo "Creating a ros2_ws"
        conda activate ros_env
        conda install colcon-common-extensions rosdep
        # Creating a ros2_ws
        mkdir ~/ros2_ws
        cd ~/ros2_ws
        mkdir src 
        colcon build
        ;;
    *)
        echo "ros2_ws couldn't be setup"
        exit 1
        ;;
esac

# Copying Mario's folders to ros2_ws

# Installig Gazebo
case "${unameOut}" in
    Linux*)
        echo "Installing Gazebo"
        curl -sSL http://get.gazebosim.org | sh
        ;;
    Darwin*)
        echo "Installing Gazebo"
        conda activate ros_env
        curl -ssL http://get.gazebosim.org | sh
        ;;
    *)
        echo "Gazebo couldn't be setup"
        exit 1
        ;;
esac

# Copying Mario's folders to ros2_ws

cd ~/ros2_ws/src
if [[ ! -d "1_chatter_listener" ]]; then
    mv /tmp/ros2_ws/1_* $HOME/ros2_ws/src
	mv /tmp/ros2_ws/2_* $HOME/ros2_ws/src
	mv /tmp/ros2_ws/3_* $HOME/ros2_ws/src
	mv /tmp/ros2_ws/4_* $HOME/ros2_ws/src
    mv /tmp/ros2_ws/activities $HOME/ros2_ws/src
    if [[ ! -d "$HOME/ros2_ws_firmware" ]]; then
        mkdir -p $HOME/ros2_ws_firmware
        mv /tmp/ros2_ws/firmware/* $HOME/ros2_ws_firmware
        echo "${red}======================"
        echo "$green Ros Repository cloned newly and processed : ESP32 Codes $reset"
    else 
        echo "${red}======================"
        echo "$green Already processed : ESP32 Codes $reset"
    fi
    rm -rf /tmp/ros_ws
        echo "${red}======================"
        echo "$green Ros Repository newly cloned and processed$reset"
    else
        echo "${red}======================"
        echo "$green Ros Repository already existed and processed $reset"
    fi
    cd ..
    colcon build


# Setting up microros_ws
case "${unameOut}" in
    Linux*)
        echo "Creating a microros_ws"
        mkdir -p microros_ws/src
        cd microros_ws/src
        git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
        sudo apt update && rosdep update
        cd ..
        rosdep install --from-paths src --ignore-src -y
        pip3 install catkin_pkg lark-parser colcon-common-extensionsls -l rosdep
        ;;
    Darwin*)
        echo "Creating a microros_ws"
        mkdir -p microros_ws/src 
        cd microros_ws/src
        git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
        
        # Pending

        ;;
    *)
        echo "micros_ws couldn't be setup"
        exit 1
        ;;
esac
