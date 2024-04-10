!/usr/bin/env bash

set -e

red=`tput setaf 1`
green=`tput setaf 2`
blue=`tput setaf 4`
reset=`tput sgr0`

echo "Installing ESP IDF"

_shell_="${SHELL#${SHELL%/*}/}"
# Check whether esp-idf has already been installed
if [ -d $HOME/esp/esp-idf ]; then
    echo "${red}======================$reset"
    echo "You already have installed esp-idf!"
    echo "${blue}======================$reset"
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
                echo "${red}======================$reset"
                echo "Homebrew is already installed"
                echo "${blue}======================$reset"
            else 
                echo "installing homebrew"
                /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
                echo "${blue}======================$reset"
                echo "homebrew installed successfully"
                echo "${blue}======================$reset"
            fi
            brew install git cmake ninja dfu-util python3  
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
    echo "${red}======================$reset"
    echo "${red}======================$reset"
    echo "${blue}Esp Installation Successfull $reset"
    # Set IDF Alias
    echo "alias get_idf='. $HOME/esp/esp-idf/export.sh'" >> $HOME/."$_shell_"rc
    get_idf
    idf.py --version 
fi

# Clone Mario repository if not already cloned
if [ ! -d "$HOME/MARIO" ]; then
    echo "${blue}======================$reset"
    echo "Cloning Mario"
    echo "${red}======================$reset"
    cd $Home
    git clone -b ros2-dev --recursive https://github.com/SRA-VJTI/MARIO.git 
    echo "${blue}======================$reset"
    echo "Mario repo cloned successfully"
    echo "${red}======================$reset"
else
    echo "Mario repository already exists and is not empty. Skipping cloning."
fi

# Check the operating system
unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)
        echo "Checking if ROS 2 is installed..."
        # Check if ROS 2 is already installed
        if ! command -v ros2 &>/dev/null; then   
            echo "${red}======================$reset"
            echo "ROS 2 is not installed. Proceeding with installation..."
            
            # Check for UTF-8 locale
            locale
            sudo apt update && sudo apt install locales
            sudo locale-gen en_US en_US.UTF-8
            sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
            export LANG=en_US.UTF-8
            locale  # verify settings
            
            # Add ROS 2 repository
            sudo apt install software-properties-common
            sudo add-apt-repository universe
            sudo apt update && sudo apt install curl -y
            sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
            sudo apt update
            sudo apt upgrade
            
            # Install ROS 2
            sudo apt install ros-humble-desktop
            echo "source /opt/ros/humble/setup.bash" >> $HOME/."$_shell_"rc

            # Install additional ROS 2 packages 
            echo "Installing Additional Ros2 packages"
            sudo apt install -y ros-humble-desktop-full ros-humble-control-msgs ros-humble-control-toolbox ros-humble-gazebo-ros2-control ros-humble-joint-state-broadcaster ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-forward-command-controller ros-humble-robot-state-publisher ros-humble-gazebo-ros2-control ros-humble-robot-controllers ros-humble-robot-controllers-interface ros-humble-robot-controllers-msgs ros-humble-joint-trajectory-controller ros-humble-controller-manager ros-humble-controller-manager-msgs
            echo "${red}======================$reset"
            echo "ROS 2 installed successfully."
            echo "${red}======================$reset"
        else
            echo "${red}======================$reset"
            echo "ROS 2 is already installed."
            echo "${red}======================$reset"
        fi
        ;;
    Darwin*)
            # Installing mambaforge
            echo "Installing mambaforge"
            if command -v mamba &>/dev/null; then
                echo "${blue}======================$reset"
                echo "Mambaforge is already installed"
                echo "${red}======================$reset"
            else
                wget -q https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh -O mambaforge.sh
                chmod +x mambaforge.sh
                ./mambaforge.sh 
                rm mambaforge.sh
                echo "${blue}======================$reset"
                echo "Mambaforge installed"
                echo "${red}======================$reset"
                echo "Initializing Mambaforge"
                export PATH="$HOME/mambaforge/bin:$PATH"
                mamba init --all
                echo "$green Mambaforge initialized. ReOpen a new terminal, and Please re-run the installation script for further configuration"
                # exit 0
            fi
            
            # Install mamba if not installed already
            conda install mamba -c conda-forge
            mamba create -n ros_env -c conda-forge 
            source $HOME/mambaforge/etc/profile.d/conda.sh
            conda activate ros_env
            conda config --env --add channels conda-forge
            conda config --env --add channels robostack-staging
            conda config --env --remove channels defaults || true
            # Install ROS packages
            mamba install ros-humble-desktop-full
            mamba install -n ros_env -y ros-humble-control-msgs ros-humble-control-toolbox ros-humble-joint-state-broadcaster ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-forward-command-controller ros-humble-robot-state-publisher ros-humble-controller-manager ros-humble-controller-manager-msgs ros-humble-joint-trajectory-controller
            mamba install catkin_tools
            echo "${red}======================$reset"
            echo "ROS 2 installed successfully."
            echo "${red}======================$reset"
        ;;
    *)
        echo "ROS 2 installation is not supported on this operating system."
        exit 1
        ;;
esac

# Verify if ros2_ws already exists
if [ -d "$HOME/ros2_ws" ]; then
    echo "ros2_ws already exists."
else
    case "${unameOut}" in
        Linux*)
            echo "${red}======================$reset"
            echo "Creating a ros2_ws"
            echo "${red}======================$reset"
            sudo apt update 
            sudo apt install python3-colcon-common-extensions rosdep
            echo "source  /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> $HOME/."$_shell_"rc
            mkdir $HOME/ros2_ws
            cd $HOME/ros2_ws
            mkdir src 
            colcon build
            echo "source  $HOME/ros2_ws/install/setup.bash" >> $HOME/."$_shell_"rc
            ;;
        Darwin*)
            echo "${blue}======================$reset"
            echo "Creating a ros2_ws"
            echo "${red}======================$reset"
            source $HOME/mambaforge/etc/profile.d/conda.sh
            conda activate ros_env
            conda install colcon-common-extensions rosdep
            # Creating a ros2_ws
            mkdir $HOME/ros2_ws
            cd $HOME/ros2_ws
            sudo chown -R $(whoami) $HOME/ros2_ws
            mkdir src 
            colcon build
            echo "${red}======================$reset"
            echo "ros2_ws successfully setup"
            echo "${red}======================$reset"
            ;;
        *)
            echo "ros2_ws couldn't be setup"
            exit 1
            ;;
    esac
fi

# Installig Gazebo if not already installed
if [ "$unameOut" == "Linux" ]; then
    if ! command -v gazebo &> /dev/null; then
        echo "${red}======================$reset" 
        echo "Installing Gazebo"
        echo "${red}======================$reset"
        curl -sSL http://get.gazebosim.org | sh
        echo "${red}======================$reset"
        echo "Gazebo installed successfully"
        echo "${red}======================$reset"
    else
        echo "${red}======================$reset"
        echo "Gazebo is already installed"
        echo "${red}======================$reset"
    fi
fi

# Copying Mario's folders to ros2_ws

cd $HOME/ros2_ws/src
if [[ ! -d "1_chatter_listener" ]]; then
        echo "${red}======================$reset"
        echo "Copying Mario's folders to ros2_ws"
        echo "${red}======================$reset"
    cp $HOME/MARIO/1_* $HOME/ros2_ws/src
	cp $HOME/MARIO/2_* $HOME/ros2_ws/src
	cp $HOME/MARIO/3_* $HOME/ros2_ws/src
	cp $HOME/MARIO/4_* $HOME/ros2_ws/src
    cp $HOME/MARIO/activities $HOME/ros2_ws/src
    if [[ ! -d "$HOME/ros2_ws_firmware" ]]; then
        mkdir -p $HOME/ros2_ws_firmware
        cp $HOME/MARIO/firmware/* $HOME/ros2_ws_firmware
        echo "${red}======================$reset"
        echo "$green firmware copied to ros2_ws_firmware $reset"
    else 
        echo "${red}======================$reset"
        echo "$green Already copied $reset"
    fi
    echo "${red}======================$reset"
else
    echo "${red}======================$reset"
    echo "$green Ros folders are already copied $reset"
fi
cd ..
source $HOME/mambaforge/etc/profile.d/conda.sh
conda activate ros_env
colcon build
echo "${red}======================$reset"
echo "Successfully copied mario's folders to ros2_ws"
echo "${red}======================$reset"

# Setting up microrosagent
case "${unameOut}" in
    Linux*)
        echo "Cloning microrosagent"
        cd $HOME/ros2_ws/src
        cd ..
        pip3 install catkin_pkg lark-parser colcon-common-extensions -l rosdep
        rosdep install --from-paths src --ignore-src -y
        echo "${red}======================$reset"
        echo "Done with cloning"
        echo "${red}======================$reset"
        ;;
    Darwin*)
        echo "${red}======================$reset"
        echo "Cloning microrosagent"
        echo "${red}======================$reset"
        cd $HOME/ros2_ws/src
        pip3 install catkin_pkg lark-parser 
        git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
        echo "${red}======================$reset"
        echo "Done with cloning microrosagent"
        echo "${red}======================$reset"
        ;;
    *)
        echo "micros_ws couldn't be setup"
        exit 1
        ;;
esac
