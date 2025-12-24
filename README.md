# Unitree Go2 Introductory Project
Test branch 1 and merge testBranch1 into master

This project is suitable for students with some basic knowledge who want to get started with Unitree Go2. It can be completed in about 2 months and has a moderate level of 
difficulty. It can help you quickly master the basics of robot dog ROS2 development.

Bilibili address:https://www.bilibili.com/video/BV1caWQzdE3G/?spm_id_from=333.337.search-card.all.click&vd_source=4bd0448ccc277efab1a6915315abd6b9

## 🛠️ Hardware Preparation  
1.Unitree Go2 EDU Edition Robot Dog  
2.PC host with ROS2 Humble installed  
3.5-10 meter network cable (for connecting the robot dog to the PC)  

## ✅ Feature implemented  
-Visualization of the robot dog model in Rviz2  
-Point cloud accumulation and`PointCloud2_to_LaserScan`Message Transformation  
-Supports official ROS2 keyboard control node  
-Based on IMU fused odometry (odom) data  
-Integrating slam-toolbox to implement mapping functionality  

## 🚧 Features to be completed  
-Autonomous navigation function (core features yet to be developed)  

## 🔧 Development Environment Configuration  

We recommend referring to the following tutorial to complete the network environment setup (**Setting up the network environment is very important and must be completed.**）：  
-Courses taught by Professor Zhao Xuzuo:[ROS2: From Beginner to Expert](https://www.bilibili.com/video/BV1vv5YzBEQH?spm_id_from=333.788.videopod.episodes&vd_source=4bd0448ccc277efab1a6915315abd6b9&p=5)(More suitable for beginners)  
-Utree official documentation:[Go2 Developer Quick Start](https://support.unitree.com/home/zh/developer/Quick_start)  

## 📦 Dependency Installation  
1. Install robot positioning fusion package  
   ```bash
   sudo apt update && sudo apt install ros-humble-robot-localization
   ```  

2. Install mapping tools  
   ```bash
   sudo apt update&& sudo apt install ros-humble-slam-toolbox
   ```  

3. Other dependencies: If the compiler prompts that packages are missing, you can use the missing packages according to the error message.`sudo apt install ros-humble-<missing package name>`Install


## 🚀 Quick Start Steps  

1. **Create and enter a workspace**  
   ```bash
   mkdir -p go2_ws_toolbox/src && cd go2_ws_toolbox/src
   ```  

2. **Cloning repository**  
   ```bash
   git clone git@github.com:FishPlusDragon/unitree-go2-slam-toolbox.git
   ```  

3. **Compiler workspace**  
   ```bash
   cd .. && colcon build
   ```  

4. **Startup function**  
   -Load environment variables  
     ```bash
     source install/setup.bash
     ```  
   - Start SLAM mapping (including visualization)  
     ```bash
     ros2 launch go2_core go2_start.launch.py
     ```  
   - Start the new terminal with keyboard control (to control the robot dog's movement and mapping).  
     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard
     ```  


>⚠️ Note: When creating a map, it is recommended to adjust the movement speed to around 0.3m/s and select "Classic Mode" for gait to ensure stability.


## Notice:
Due to the author's academic pressure, this project is no longer maintained. However, if there are any bugs or if any experts have fixed the navigation, please share them in the issues so I can learn from them.

## Acknowledgements:
- Thank you to everyone who has helped me on my learning journey.
