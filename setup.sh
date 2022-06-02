#!/bin/bash

# TODO: Automate the installation of ROS
function installROS () {
    echo "Hello World"
}

# TODO: Automate the installation of CrazySwarm
function installCrazySwarm () {
    echo "Hello World"
}

function installNode () {
    echo "[*] Installing NodeJS"

    # Add 1 tab for a better stdout
    exec 3>&1
    exec 1> >(paste /dev/null -)

    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    
    sudo apt-get install -y nodejs
    RET_VALUE=$(node --version)

    echo "[x] Successfully installed NodeJS $RET_VALUE"

    echo "[*] Installing NPM"
    sudo apt install npm
    RET_VALUE=$(npm --version)

     # Reset the 1 tab setting
    exec 1>&3 3>&-
    
    echo "[x] Successfully installed NPM $RET_VALUE"
}

# TODO: Automate the installation of Redis-Server and Redis-Cli
function installRedis() {
    echo "[*] Installing Redis"

    # Add 1 tab for a better stdout
    exec 3>&1
    exec 1> >(paste /dev/null -)

    sudo apt install redis-server

    # Reset the 1 tab setting
    exec 1>&3 3>&-

    echo "[x] Successfully installed redis-server and redis-cli"
}

# Automate the installation of Python3 packages
function installPythonPackages () {
    echo "[*] Installing the Python3 packages"

    # Add 1 tab for a better stdout
    exec 3>&1
    exec 1> >(paste /dev/null -)

    python3 -m pip install redis
    python3 -m pip install flask
    python3 -m pip install flask-cors

    # Reset the 1 tab setting
    exec 1>&3 3>&-

    echo "[x] Successfully installed the Python3 packages"
}

# TODO: Check the current status of the system
# TODO: For examaple: whether the ROS is installed or not and etc.

# Install NodeJS and NPM
installROS
installCrazySwarm
installNode
installRedis
installPythonPackages