# DJI-M100-Drone-Based-Tracking
A ROS Package that achieved visual based object tracking.

## Hardware

DJI Matrice 100 + Manifold (Nvidia Tegra K1) + Zenmuse X5 Gimbal.

## Enviroment

Ubuntu 14.04 (provided by Nvidia);

ROS Indigo;

OpenCV 2.4;

DJI gimbal camera lib;

## Tracking

Kernelized correlation filter was implemented as the visual tracker. Plus, an object loss detection kit was added to the tracker. A frame-difference method was added but was initially not activited (for some bugs).

## Structure

visual_tracker.cpp: read cam data, execute tracker.

mission_planner.cpp: controller.

flight_controller.cpp: main function, send control signal to the DJI flight controller.

## result

The result was carried in Zhijiang campus, Zhejiang Univ.. Target lost at last.

![image](https://github.com/SuboOx/DJI-M100-Drone-Based-Tracking/blob/master/result.gif)

