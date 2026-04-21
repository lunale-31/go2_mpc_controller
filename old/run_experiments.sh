#!/bin/bash

COLOR_YELLOW="\033[33m"
COLOR_RESET="\033[0m"

run_joint_experiments() {
    echo -e "${COLOR_YELLOW}Running $1 experiments${COLOR_RESET}"
    ./square-wave "../config/$1.yml"
    ./sine-wave "../config/$1.yml"
    ./cascaded-sine-wave "../config/$1.yml"
}

run_experiments() {
    WORKDIR="$(pwd)"
    BUILDDIR="$(dirname "$0")/build/"
    cd $BUILDDIR
    
    echo "Building binaries"
    cmake ..
    make all

    run_joint_experiments "hip"
    run_joint_experiments "thigh"
    run_joint_experiments "calf"

    echo "Done!"
    cd $WORKDIR
}

echo -e "${COLOR_YELLOW}Warning: The robot will perform a series of rapid motions.${COLOR_RESET}"
echo -e "Make sure the robot is firmly placed on a test stand and that the ${COLOR_YELLOW}BACK-RIGHT${COLOR_RESET} leg can move freely in all directions."
echo
read -p "Is the robot in a safe position? [yN] " confirm
if [ "$confirm" == "y" ]; then
    echo "Starting test run."
    run_experiments
else
    echo "Aborting."
fi