# <div align="center">Autonomous Drifting</div>
![image](Media/UCSDLogo_JSOE_BlueGold_0_0.png)
## <div align="center"> 148-winter-2025-final-project-team-6</div>

---

## Table of Contents
1. [Team Members](#team-members)
2. [Abstract](#abstract)
3. [What We Promised](#what-we-promised)
4. [Accomplishments](#accomplishments)
5. [Challenges](#challenges)
6. [Final Project Videos](#final-project-videos)
7. [Software](#software)
8. [Hardware](#hardware)
9. [Gantt Chart](#gantt-chart)
10. [Course Deliverables](#course-deliverables)
11. [Project Reproduction](#project-reproduction)
12. [Acknowledgements](#acknowledgements)
13. [Contacts](#contacts)

---

## Team Members
- Belle Li — Mechanical Engineering (MC27)  
- Sidhant — Electrical and Computer Engineering  
- Amara — Electrical and Computer Engineering  
- Abhinav — Mechanical Engineering  

---

## Abstract
Our project focused on enabling **autonomous drifting** using a robotic platform equipped with integrating the **OAK-D camera** and the inbuilt **VESC IMU**. The vehicle follows yellow lines for path tracking and initiates controlled drifts when it detects blue lines on the track. Drift quality, direction, and response were monitored and analyzed using real-time gyroscope and accelerometer data.

---

## What We Promised

### Must-Haves
- Enable controlled drifting  
- Enable autonomous drifting using OAK-D vision input  
- Use IMU data to track drift initiation, speed, and direction  

### Nice-to-Haves
- Enable simultaneous detection of yellow and blue using OpenCV  
- Use real-time IMU feedback to improve drift performance  

---

## Accomplishments
- Developed a custom ROS2 node that performs multi-color line detection using OAK-D  
- Tuned the PID control system to transition between tracking and drifting  
- Integrated real-time gyroscope and accelerometer data from the VESC IMU  
- Visualized drift characteristics using yaw rate and acceleration data  
- Delivered reliable drifting and recovery maneuvers autonomously  

---

## Challenges
- ❌ Artemis IMU communication issues led to unreliable data  
- ⚠️ High-speed turns caused detection lag in some frame drops  
- 💡 Solved IMU issues by switching to VESC's onboard IMU and modifying firmware  

---

## Final Project Videos
> 

---

## Software
- ROS2 Foxy  
- Python3 (OpenCV, NumPy, rclpy)  
- OAKD for camera integration  
- VESC Tool for IMU & motor feedback  
- Main custom ROS2 nodes:
  - `lane_detection_node` (Yellow + Blue path)
  - `lane_guidance_node` (PID + twist publishing)
  - `imu_processing_node` (Yaw, Accel, Drift logging)

---

## Hardware
- OAK-D Lite stereo camera  
- VESC with integrated IMU  
- RC Car (modified for drift performance)  
- Jetson Nano (on-board compute)  
- 3D-printed mounts for stability  
- Logitech F710 Game Controller (teleop debugging)

---

## Project Reproduction
1. Clone the repository:
    ```bash
    git clone https://github.com/UCSD-ECEMAE-148/148-winter-2025-final-project-team-6.git
    ```
2. Follow instructions in `ros2_ws/src/README.md` for building and launching  
3. Set calibration parameters in `ros_racer_calibration.yaml`  
4. Run:
    ```bash
    colcon build
    source install/setup.bash
    ros2 launch lane_following.launch.py
    ```

---

## Acknowledgements
- Thanks to Prof. Jack Silberman, Alexander, Winston, Vivekanand and peers.

---

## Contacts
For questions or collaboration, contact:  
- Sidhant: [ssinghvi@ucsd.edu]  
- Belle: [bel010@ucsd.edu]  
- Amara: [nihekwoeme@ucsd.edu]  
- Abhinav: [aswarup@ucsd.edu]

---
