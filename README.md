# Right-of-Way System of Robots & Yielding

A ROS Robot  Project

For detail Check the labbook report link:
https://campusrover.github.io/labnotebook2/reports/2024/RIghtsofRobots/

For desired result, check the demo video link:
https://drive.google.com/file/d/1P-VI9nCCOJ-jc9P-BzkvQxoZSchtOytG/view?usp=sharing

## Team Members

- **Haochen Lin**  
  Email: haochenlin@brandeis.edu

- **Zixin Jiang**  
  Email: zixinjiang@brandeis.edu

## Code
The github saves the essential documents in ROS package format, you can use it use copy the files to the corresponding ROS package folder.

launch folder saves all the .launch file, and src file saves all the python files. fiducial_verion is legacy content, you will not needed for this project, but if you want to run single robot fiducial recognition, this folder could be useful.


## How to Use
After connect everything either a real robot or a gaze_world simulation, you will be able to run the following files in you package.

Firstly, bringup the main robot then bringup the follower robot if you working on real robot.

For the following part, change "my_project" to the actual package name that your code saves.

1. Start scheduler:
   ```bash
   rosrun my_project scheduler.py
   ```
2. Run the communication and running module:
   ```bash
   roslaunch my_project road_run.launch
   ```
3. Reset schduler everytime you run road_run.launch file.

The above should performs desired output as the demo video shows, but mistake will likely happen if certain requirement is not satisfied.

For extra content showing at the last of the demo video:

1. Start scheduler:
   ```bash
   roslaunch my_project robot_odom.launch
   ```
2. Run the communication and running module:
   ```bash
   roslaunch my_project robot.launch
   ```

---


