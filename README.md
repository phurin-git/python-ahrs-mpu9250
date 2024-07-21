# Raspberry Pi AHRS with MPU9250

This repository contains a project that leverages a Raspberry Pi to implement an Attitude and Heading Reference System (AHRS) using the MPU9250 sensor. The project includes several Python scripts that handle various aspects of the AHRS system, including sensor data acquisition and processing.
## Components
**ahrs_mpu9250.py**
- Integrates data from the MPU9250 sensor.
- Computes orientation using sensor data.

**mpu9250.py**
- Interfaces with the MPU9250 sensor. 
- Handles sensor data reading and initial processing.

**registermap.py**
- Provides register map definitions for the MPU9250 sensor.
- Facilitates easier and more readable register access.
## Usage
1. Connect the MPU9250 sensor to the Raspberry Pi.

2. Run the asrs.py script to start the AHRS system:
```bash
python ahrs_mpu9250.py
```


## Related

Here is the related projects
- [AHRS: Attitude and Heading Reference Systems](https://ahrs.readthedocs.io/)
- [IMU-Based Motion Capture Using Madgwick Filter with 3D Visualization for Robot Teleoperations](https://ieeexplore.ieee.org/document/10589761)
- [Development of a Low-Cost Manipulator for Wireless Control from Upper Limb Motion Capture System](https://ieeexplore.ieee.org/document/10589750)