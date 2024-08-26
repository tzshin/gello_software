# GELLO with Mods and More Instructions
## Introduction
This repository provides comprehensive information on using the GELLO with the Franka Emika robot arm, including various modifications and improvements we have made to enhance efficiency and user-friendliness.  
**Note:** This repository is intended to be used alongside the original GELLO repository, as some necessary information can be found there.
## Table of Contents
- [Motivation](#motivation)
- [Setup](#setup)
  - [Hardware Requirements](#hardware-requirements)
  - [Software Installation](#software-installation)
- [Calibrating Offset](#calibrating-offset)
  - [Manual Calibration](#manual-calibration)
  - [Automatic Calibration](#automatic-calibration)
- [Running Simulation](#running-simulation)
- [Running on Real Robot](#running-on-real-robot)
  - [Without Automatic Offset Calibration](#without-automatic-offset-calibration)
  - [With Automatic Offset Calibration](#with-automatic-offset-calibration)
- [Data Collection](#data-collection)
- [Modifications and Improvements](#modifications-and-improvements)
## Motivation
The original GELLO repository provides limited information on using GELLO’s software, with some problems in the provided code. This repository addresses these issues by offering additional insights and enhancements to improve the user experience.
## Setup
### Hardware Requirements
- Intel NUC with a real-time OS
- Franka Emika robot arm
- Network switch
- Separate computer with Ubuntu installed
### Software Installation
1. **Install Polymetis and GELLO:**
   - Connect the Intel NUC to Franka and log into Franka Desk.
   - Install Polymetis on the NUC.
   - Connect a separate Ubuntu-equipped computer to the NUC via a network switch and install Polymetis and GELLO on this computer.
2. **Create a Conda Environment:**
   - We recommend installing GELLO in a Conda environment due to issues with the Docker container.
   - Follow the instructions in the original GELLO repository to install Polymetis and GELLO.
**Important:** Use a separate NUC to control the robot, as recommended by Polymetis. We encountered issues running GELLO on a single machine.
### Troubleshooting
- If you encounter the error `cannot find torchscript_pinocchio` while running Polymetis, creating a new user account on the machine may resolve the issue.
## Calibrating Offset
### Manual Calibration
The hardware consists of multiple servos acting as joint angle encoders, which require recalibration before each session. Use the following commands to calibrate the offset manually:
1. **Recommended Initial Configuration:**

<p align="left">
  <img src="https://github.com/tzshin/gello_software/blob/main/imgs/gello_home.png" width="200">
</p>

```
python scripts/gello_get_offset.py \
--start-joints 0 0 0 -1.571 0 1.571 0 \
--joint-signs 1 -1 1 1 1 -1 1 \
--port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISXHW-if00-port0
```
2. **Original Configuration:**
Put the arm upright, with trigger touching the arm.
```
python scripts/gello_get_offset.py \
--start-joints 0 0 0 0 0 0 0 \
--joint-signs 1 -1 1 1 1 -1 1 \
--port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISXHW-if00-port0
```
After obtaining the offset values, update `gello/agents/gello_agent.py` with these values, including the gripper angles and port ID.
### Automatic Calibration
Use the provided shell scripts to automate offset calibration and streamline the process.
## Running Simulation
We recommend running GELLO in simulation mode before using it on the real robot. Follow the original repository’s instructions to set up the simulation.
**Note:** The gripper joint on Franka has a slight offset in the 0-degree position. This will affect the alignment in the simulation and the actual robot.
## Running on Real Robot
### Without Automatic Offset Calibration
1. **On the NUC:**
```
launch_robot.py robot_client=franka_hardware
launch_gripper.py gripper=franka_hand
```
2. **On the Ubuntu Computer:**
```
python3 experiments/launch_nodes.py --robot panda --robot_ip="NUC’s IP address"
python3 experiments/run_env.py --agent=gello
```
### With Automatic Offset Calibration
1. **On the NUC:**
```
launch_robot.py robot_client=franka_hardware
launch_gripper.py gripper=franka_hand
```
2. **On the Ubuntu Computer:**
```
python3 experiments/launch_nodes.py --robot panda --robot_ip="NUC’s IP address"
./automation/run_gello.sh
```
Ensure GELLO is in the same position as the Franka arm before running the final command. The Franka robot will go to its home position after running `launch_nodes.py`.

<p align="left">
  <img src="https://github.com/tzshin/gello_software/blob/main/imgs/franka_home.png" width="200">
</p>

## Data Collection
To collect data, follow the steps for automatic offset calibration, but use the following command:
```
./automation/run_gello_with_data_collection.sh
```
A PyGame window will appear. Press `s` to start recording data and `q` to stop. The data will be saved automatically in a folder named `bc_data` in your home directory.
## Modifications and Improvements
- **Automated Offset Calibration:**
  - **Scripts:**
    - `automation/run_gello.sh`
    - `automation/run_gello_with_data_collection.sh`
    - `scripts/gello_get_offset.py`
    - `gello/agents/gello_agent.py`
  - We automated the offset calibration process to reduce preparation time and enhance accuracy. The retrieved offset values by `scripts/gello_get_offset.py` are saved to a JSON file, which is read by `gello/agents/gello_agent.py` during startup. The bash scripts will run the offset retrieval script before starting teleoperation.
- **Error Handling:**
  - Improved error handling for the gripper trigger’s position to ensure successful teleoperation startup. The bash interactively lets the user adjust the gripper trigger position and press ‘enter’ again to retry starting teleoperation.
- **Software Improvements from Commit [`5561acb`](https://github.com/tzshin/gello_software/commit/5561acbda948b69182ed21209da5258feddd4493):**
  - **Enhanced Calibration Procedure:**
    - The new calibration procedure introduced in `scripts/gello_get_offset.py` significantly improves the precision of joint angle measurements. Changes ensure that the joint offsets are correctly updated and stored in a JSON file for seamless integration and reuse.
  - **Gripper Sticking Issue Resolution:**
    - The modifications in the `gello/robots/panda.py` script address the gripper sticking issue. A state-machine has been implemented to handle the gripper's grasping state, enhancing reliability during operation. This state-machine effectively manages transitions between different gripper states (free, stalled, grasping), ensuring smoother operation and preventing the gripper from stalling during critical tasks.
- **Reference to External Polymetis Issue:**
  - A similar gripper sticking issue was identified and discussed in a Polymetis community on GitHub, particularly in the context of using the Polymetis framework with a Franka gripper. Although the fixes proposed in [PR #1417 on the Fairo repository](https://github.com/facebookresearch/fairo/pull/1417) did not resolve the issue completely for real robots, the insights were beneficial. This PR introduced a new API, `GripperInterface.stop`, to preemptively stop the gripper to resolve sticking issues caused by previous commands. This approach is akin to strategies we've explored in our own developments with GELLO and should be considered when encountering similar issues.  

By following the instructions in this README, users can efficiently set up and use the teleoperation tool for the Franka robot arm. For further information and troubleshooting, please refer to the original GELLO repository.
