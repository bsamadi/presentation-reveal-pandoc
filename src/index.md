---
author: Behzad Samadi
title: Virtual Prototyping
date: April 9, 2014
---

## Virtual Prototyping

Behzad Samadi, PhD  
[Mechatronics3D](http://www.mechatronics3D.com)  
DIPECC 2014, Dubai  
April 9, 2014

# It All Started with LEGO!

## LEGO MINDSTORMS NXT

[![LEGO MINDSTORMS](./media/LEGO/LEGOMINDSTORMSNXT.jpg)](http://mindstorms.lego.com)  
[LEGO^®^](http://mindstorms.lego.com)

## LEGO MINDSTORMS EV3

[![LEGO^®^](./media/LEGO/EV3brickwMotorsSensors.jpg)](http://mindstorms.lego.com)  
[LEGO^®^](http://mindstorms.lego.com)

## Visual Programming

NXT-G  
[![Geek Dad](./media/LEGO/myblocks.png)](http://geekdad.com/2013/08/hands-on-ev3-mindstorms/)  
[LEGO^®^](http://mindstorms.lego.com)

## Open Software

[![image](./media/LEGO/LEGOsoftware)](./media/LEGO/LEGOsoftware.svg)

## Open Hardware

[![image](./media/LEGO/LEGOhardware)](./media/LEGO/LEGOhardware.svg)

## Bicycle

[![](./media/LEGO/LEGObicycle.png)](./media/LEGO/LEGObicycle.mp4)  
Razyeh Mousavi

## Ballbot

[![](./media/LEGO/LEGOballbot.jpg)](https://www.youtube.com/watch?v=f8jxGsg3p0Y)

## NXTway

[![](./media/LEGO/LEGOnxtway.png)](./media/LEGO/LEGOnxtway.mp4)  
[Hamid Bazargani](http://engcast.com/main/)  
[NXTway-GS](http://lejos-osek.sourceforge.net/videos.htm)

## Ball on Plate

[![](./media/LEGO/LEGOballonplate.jpg)](https://www.youtube.com/watch?v=_YILNkytBjE&list=PLDB2B51106611C1E5)

## Electronics Stability Control

[![](./media/LEGO/LEGOcar.png)](./media/LEGO/LEGOesc.mp4)  
[Reza Azimi](http://brown.edu/research/labs/engineering-man-machine-systems/systems/people/students/razimi)  
[NXTway-GT](http://lejos-osek.sourceforge.net/videos.htm#NXT_GT_Hi)  

## LEGO MINDSTORMS

It was very exciting at first but I had more than 40 students and only 3 sets of LEGO Mindstorms!

## Amazing Journey

We realized that before making the robot, there were a lot of things that we needed to learn.

## Operating System

[![](./media/LEGO/LEGOosek.png)](http://lejos-osek.sourceforge.net/)

## Virtual Robot

[![](./media/LEGO/NXTway-GS.jpg)](http://lejos-osek.sourceforge.net/)

## 3D Animation

[![](./media/LEGO/NXTway-GS-Viewer.jpg)](http://lejos-osek.sourceforge.net/)

## Controller Design

[![](./media/LEGO/NXTway-GS-Controller.jpg)](http://lejos-osek.sourceforge.net/)

## How About a Virtual LEGO Set?

- This virtual robot is interesting but what if I want to build another robot?
- How hard is it to build a virtual robot?

## Causal Modeling 

- Writing the dynamic equations
- Converting the equations to ordinary differential equations
- Creating a signal flow model e.g. a Simulink model
- Building a virtual robot seems to be different from building a real robot

## Virtual LEGO Set

What if we had a set of virtual LEGO parts and we could build a robot with them?

## Virtual LEGO Set

We made one!  
![](./media/LEGO/LEGOvirtual.png)  

- [ODE](http://www.ode.org/) (Open Dynamics Engine)
- [IrrLicht](http://irrlicht.sourceforge.net/) (Visualization)  

Alborz Gharraee

# Acausal Modeling

## Modelica and MapleSim 

[![](./media/Modelica/DoublePendulum.png)](http://www.maplesoft.com/products/maplesim/)

- The connections between components are physical connections like position, voltage, flow,...
- Acausal modeling is very similar to building a real system

## Electrical Systems

[![](./media/Modelica/Electrical.png)](http://www.maplesoft.com/products/maplesim/)

## Mechanical Systems

[![](./media/Modelica/Mechanical.png)](http://www.maplesoft.com/products/maplesim/)

## Hydraulic Systems

[![](./media/Modelica/Hydraulic.png)](http://www.maplesoft.com/products/maplesim/)

## Thermal Systems

[![](./media/Modelica/Thermal.png)](http://www.maplesoft.com/products/maplesim/)

## Chemical Systems

[![](./media/Modelica/Chemical.png)](http://www.control.lth.se/Research/ResearchComplexSystems/numerical-and-symbolic-algorithms-for-dynamic-optimization.html)

## Acausal Modeling with MapleSim

[![](./media/Modelica/MapleSimWorkflow.png)](http://www.maplesoft.com/products/toolboxes/FMI/index.aspx)

## Functional Mockup Interface

[![](./media/Modelica/FMI.jpg)](https://itea3.org/assets/itea/image/638.jpg)

- Model Exchange, Co-Simulation, Product Lifecycle Management
- There are over 35 [Tools](https://www.fmi-standard.org/tools) supporting FMI

Courtesy of [ITAE](https://itea3.org/)

# Model Based Development

## The V Diagram

[![](./media/MBD/Systems_Engineering_V_diagram.jpg)](http://www.engineering.com/DesignSoftware/DesignSoftwareArticles/ArticleID/7352/Model-Based-System-Engineering--Beyond-Spreadsheets.aspx)

from [engineering.com](http://www.engineering.com/DesignSoftware/DesignSoftwareArticles/ArticleID/7352/Model-Based-System-Engineering--Beyond-Spreadsheets.aspx)

## Trial and Error

![](./media/MBD/ConventionalApproach.png)    
Expensive, time consuming, not reliable

## Model Based Approach

![](./media/MBD/ModernApproach.png)

## Model Based Development

[![](./media/MBD/FMIvalidation.jpg)](https://itea3.org/assets/itea/image/639.jpg)  
Saves resources, time, money

Courtesy of [ITAE](https://itea3.org/)

## Old Approach

- Easy: build the model and design the controller in a high level environment like MATLAB\Simulink
- Hard: rewrite your code for the hardware controller on the physical prototype
- Problem: rewriting the code is time consuming and error prone

## Modern Approach

![](./media/MBD/MiLSiLHiL.png)

## Software in the Loop

![](./media/MBD/SiL.png)

- The controller and the virtual prototype are two different applications.
- The applications "talk" to each other using a "middleware".
- The controller doesn't see the difference between the virtual and physical prototypes.
- Software engineers do not have to wait for the hardware!

## Middleware

- Using the same language to exchange messages 
- Components can be replaced without affecting the whole system

##

[![](./media/MBD/Broom.jpg)](http://www.icub.org/)

##

[![](./media/MBD/Ship.jpg)](http://www.icub.org/)

##

[![](./media/MBD/Modular.jpg)](http://www.icub.org/)

# Robot Operating System

## What is ROS?

- a set of software libraries and tools 
- from drivers to state-of-the-art algorithms
- powerful developer tools
- open source.

## 

[![](./media/ROS/ROS-FiveYears.png)](./media/ROS/ROS-FiveYears.mp4)  

## PR2 by Willow Garage

[![](./media/ROS/PR2.jpg)](http://www.willowgarage.com/)  

[Willow Garage is changing](http://spectrum.ieee.org/automaton/robotics/robotics-software/willow-garage-to-shut-down)

## TurtleBot

[![](./media/ROS/TurtleBot.jpg)](http://www.clearpathrobotics.com/turtlebot_2/)  

## Husky

[![](./media/ROS/Husky.jpg)](http://www.clearpathrobotics.com/husky/)  

[Clearpath Robotics](http://www.clearpathrobotics.com/), Waterloo, Ontario

## Baxter

[![](./media/ROS/Baxter.jpg)](http://www.rethinkrobotics.com/)  

[list of robots](http://wiki.ros.org/Robots)

## ROS Industrial and MoveIt

[![](./media/ROS/moveit-title-small.png)](http://moveit.ros.org)  

[![](./media/ROS/MoveIt!Montage 2013.png)](./media/ROS/MoveIt!Montage 2013.mp4)  

[list of robots](http://moveit.ros.org/robots/)

[src](https://www.youtube.com/watch?v=dblCGZzeUqs) 


## NXT ROS

[![](./media/ROS/ProcessingNXT.gif)](https://code.google.com/p/brown-ros-pkg/wiki/ROSProcessingjsNXT)  

[brown-ros-pkg](https://code.google.com/p/brown-ros-pkg/wiki/ROSProcessingjsNXT)

## ROS 101

[![](./media/ROS/ros101-1.png)](http://www.clearpathrobotics.com/blog/how-to-guide-ros-101/)  

## ROS 101

[![](./media/ROS/ros101-2.png)](http://www.clearpathrobotics.com/blog/how-to-guide-ros-101/)  

## ROS 101

[![](./media/ROS/ros101-3.png)](http://www.clearpathrobotics.com/blog/how-to-guide-ros-101/)  

[more](http://www.clearpathrobotics.com/blog/how-to-guide-ros-101/)

## Camera

[![](./media/ROS/OpenCV_Gazebo_ROS.png)](./media/ROS/OpenCV_Gazebo_ROS.mp4)  

[Mike Charikov](https://www.youtube.com/watch?v=_8AhNWKzv2k) 

## OpenCV

[![](./media/ROS/opencv_overview.jpg)](http://opencv.org/)  

## OpenCV

[![](./media/ROS/trackwe.png)](https://code.google.com/p/opencv-lane-vehicle-track/)   

motion, line, face, feature detection

## Depth Estimation with OpenCV

[![](./media/ROS/DepthEstimation.png)](./media/ROS/DepthEstimation.mp4)  

[src](https://www.youtube.com/watch?v=LbtTsKiSQkE) 

## Gesture Detection with OpenCV

[![](./media/ROS/GestureOpenCV.png)](./media/ROS/GestureOpenCV.mp4)  

[src](https://www.youtube.com/watch?v=B4dwu3si9x0) 


## CloudSim

[![](./media/ROS/CloudSim.png)](http://cloudsim.io)  

# Resources

## Bookmarks

- [MapleSim](http://www.maplesoft.com/products/maplesim/)
- [Clearpath Robotics](http://www.clearpathrobotics.com/)
- [Open Source Robotics Foundation](http://osrfoundation.org/)
- [ROS](http://www.ros.org/)
- [ROScon](http://roscon.ros.org/)
- [ROS Cheat Sheet](http://www.clearpathrobotics.com/wp-content/uploads/2014/01/ROS-Cheat-Sheet-v1.01.pdf)
- [Gazebo](http://gazebosim.org/)
- [Orocos](http://www.orocos.org/)
- [MORSE](http://www.openrobots.org/wiki/morse/)
- [JdeRobot](http://jderobot.org/)
- [The Robotics Challenge](http://www.theroboticschallenge.org/)
- [Rethink Robotics](http://www.rethinkrobotics.com/)
- [Dr. Robot](http://www.drrobot.com/)
- [Best practices in robotics](http://www.best-of-robotics.org/)

## Bookmarks

- [OpenCV](http://opencv.org/)
- [Robot App Store](http://www.robotappstore.com/)
- [Robot Web Tools](http://robotwebtools.org/)
- [Robohub](http://robohub.org/)
- [Silicon Valley Robotics](http://www.svrobo.org/)
- [Suitable Technologies](https://www.suitabletech.com/)
- [Popular Science](http://www.popsci.com/category/tags/robots)
- [Robot Standards](http://www.robot-standards.eu/)
