#!/bin/bash

install_dependencies() {

    if [ -f /etc/os-release ]; then
        . /etc/os-release

        DISTRO=$ID
    else
        echo "Cannot determine the Linux distribution. Exiting."
        exit 1
    fi

    echo "Detected distribution: $DISTRO"

    # Detect if running inside WSL
    if [ -f /proc/sys/fs/binfmt_misc/WSLInterop ]; then
        echo "Running inside WSL"
        IS_WSL=true
    else
        IS_WSL=false
    fi

    if [ "$IS_WSL" = true ]; then
        echo "Detected Windows Subsystem for Linux (WSL)."

        echo "Adding NVIDIA Container toolkit for GPU Support"
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
        curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
        sudo apt-get update
        sudo apt-get install -y nvidia-container-toolkit
    fi

    # Run commands based on the detected distribution

    case "$DISTRO" in
        ubuntu)
            echo "Running Ubuntu-specific installation..."
            sudo apt update
            sudo apt install -y software-properties-common
            sudo add-apt-repository -y ppa:apptainer/ppa
            sudo apt update
            sudo apt install -y apptainer
            sudo apt install -y python3 \
                python3-pip \
                git
            ;;
        debian)
            echo "Running Debian-specific installation..."
            sudo apt update
            sudo apt install -y wget
            cd /tmp
            wget https://github.com/apptainer/apptainer/releases/download/v1.4.0/apptainer_1.4.0_amd64.deb
            sudo apt install -y ./apptainer_1.4.0_amd64.deb
            sudo apt install -y python3 \
                python3-pip \
                git
            ;;
        *)
            echo "Unknown or unsupported distribution."
            echo "Manual installation of 'apptainer' is required."
            echo "See apptainer docs: https://apptainer.org/docs/admin/latest/installation.html"
            ;;
    esac

}

setup_workspace() {
    WORKSPACE="$HOME/eolab_ws"
    mkdir -p ~/eolab_ws/src && cd ~/eolab_ws/src/
    git clone https://github.com/EOLab-HSRW/drones-ros2.git
    cd drones-ros2 && git pull origin main
    echo $PWD
    apptainer build eolab.sif apptainer.def
    apptainer exec eolab.sif bash -c "source /opt/ros/humble/setup.bash && cd ~/eolab_ws/src/drones-ros2/ && vcs import < .repos"
    apptainer exec eolab.sif bash -c "cd ~/eolab_ws/src/drones-ros2/ && eolab_drones build --type sitl --drone protoflyer --msgs-output ~/eolab_ws/src/drones-ros2/px4_msgs"
    # apptainer exec eolab.sif bash -c "source /opt/ros/humble/setup.bash && cd ~/eolab_ws/ && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y"
}

install_dependencies
# setup_workspace
