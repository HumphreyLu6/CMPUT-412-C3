# CMPUT412 FALL 2019 - competition three report
_**Purpose**_

The third competition is based on the second competition. Besides the original tasks, there are three more tasks: 1. Parking at the right AR tag. 

<img src="https://github.com/HumphreyLu6/CMPUT-412-C3/blob/master/images%20and%20video/1.jpg" width="40%" height="40%">

2. Parking at the right shape that has been identified.

<img src="https://github.com/HumphreyLu6/CMPUT-412-C3/blob/master/images%20and%20video/6.jpg" width="40%" height="40%">

3. Parking at the specific square as asked. 

<img src="https://github.com/HumphreyLu6/CMPUT-412-C3/blob/master/images%20and%20video/2.jpg" width="40%" height="40%">

The additional task is held between original task two and task three where there is an off-ramp point, there will be no more white line on the ground. The robot will be navigating by itself without following the line. There are red squares outlined as parking spots for the robots. Each square is marked with an AR tag or a shape or just unmarked. When the robot encounters a square with AR-tag it will turn green led light. If the robot encounters a square with the right shape which has been identified in location 2, it will turn orange light. Otherwise, if the square is unmarked it will turn the red light. At the beginning of each run, the TA will ask the team member to park at one specific location which should be sent to the robot by joy controller. There are eight parking spots in total. Boxes are placed to separate the new task with competition2. After the robot finishing the searching mission, a new point called ‘on-ramp’ will determine where the robot should exit the new task. The moment the robot hits the ‘on-ramp’ point, it will find the following white line in competition 2 and continue to the same last task as Competiotion2. 

<img src="https://github.com/HumphreyLu6/CMPUT-412-C3/blob/master/images%20and%20video/4.jpg" width="40%" height="40%">



## _**Pre-requisites**_

-   The project is built with python2.7 on Ubuntu 16.04. Dependencies include ROS kinetic package, smach state machine, and other drivers for the turtle bot sensor. If these are not installed please refer to the official installation page on ROS wiki or official python installation websites.

    -   Kobuki  [http://wiki.ros.org/kobuki/Tutorials/Installation/kinetic](http://wiki.ros.org/kobuki/Tutorials/Installation/kinetic)

    -   Ros-Kinetic  [http://wiki.ros.org/kinetic/Installationu](http://wiki.ros.org/kinetic/Installationu)

    -   Python2  [https://www.python.org/downloads/](https://www.python.org/downloads/)

    -   Smach  [http://wiki.ros.org/smach](http://wiki.ros.org/smach)

This project is built upon
[https://github.com/TianqiCS/CMPUT412-C2]
[https://github.com/TianqiCS/CMPUT412-C1]
Create or navigate the existing catkin workspace and clone our repository.


## _**Execution**_

-   Once you have the package in your workspace, change the package name to c3

    ```
    cd (your path)/catkin_ws
    catkin_make
    source devel/setup.bash

    ```
    now you can launch the program using

    ```
    roslaunch c3 c3_main.launch
    ```

-   arguments and parameters In the launch file c3.launch, the file will launch basic driver for the kuboki robot which is essential for the competition ( minimal.launch and 3dsensor.launch). Next, the file will bring up the basic node for this competition like main file and a usb camera. Finally, there are different sections for in the launch file like example.yaml to give the uvc camera  a basic understanding of view.

-  A map file of the lab is added to the file folder which is used for work4(the new location).   In the c3.launch file, the ar_track_alvar is used to regonize the AR tag. We comment out the view_nevigation package to increase the performance of the robot at runtime.


## _**Process and Strategy**_
-    Our basic strategy includes using pid controller to follow lines, using opencv contour shape detection to detect shapes, using amcl to do localization, using move_base to reach goal point in the location 4.
-    Here are the process details:
-    Firstly, the robot will start with "Wait" state, once the user send unmarked dock point number and start signal, the robot will start follow the white line.
-    As the robot is running, it will find out whether there is a red long line(which means stop) or a red short line(which means detecting the image) and decide if it needs to switch states.
-    For different working tasks, the difference is based on the global variable of "current_work"
-    The state machine will have some kind of work flow like this:
        - 1. Following state will keep the robot following the white line
        - 2. If the robot hit a long red line it will enter the PassThrough state to perform a stop at the long red line
        - 3. If the robot hit a short red line it will enter the TaskControl state to determine how many 90 degrees it should trun and then it goes into Rotate state which controls the robot's rotation based on the yaw value.
        - 4. In the Rotate state, the robot will determine what kind of work it will do based on current value.
        - 5. For the task to count number of white tubes, the robot will detect how many red contours are in the front and indicate the number by Led lights and sound.
        - 6. For the task at location 2, the robot will detect how many red/green contours are in the front and indicate the total number by Led lights and sound, the robot will remember what shape the green contour is in location2.
        - 7. The robot goes 'off ramp' to dock at three locations, one has a the AR tag in the front, one has a contour with same shape in location 2 in the front, one has the index specified in the biginning of the game. The robot will parks in the center of the each square/location.
        - 9. After the robot finishs all parking task it will go to the 'on ramp' point and continue the 'lcoation 3' task, which is find the matching shape at location 2.
        - 10. The robot will go through all the shapes when selecting the shapes. If it found the right one it will make a turn on a light and make a sound.
        - 11. The run is ended when the robot is back to the starting line

![Fig1](smach.png)


#### Notes:

Strategy for camera:

-    We put additional usb camera at the lower front of the turtle_bot to follow the white line on the ground and the asus camera is used to detect shape of the target. The lower position of the camera improves precision with less exception tolerance as a trade-off.
-    In the function usb_callback, we use the usb camera to detect whether we have a long red line to  short red line. The method is that if it is a long red line there won't be any white in the middle of the track. We think its quicker and easier to identify the difference between two lines.
-    If the object cannot be included into the camera, the robot can fall up a little bit to fit the camera view into the right position.

Image detection process:

-    Used cv2.pyrMeanShiftFiltering to blur image when detect contours' shapes, but this caused lag.
-    To ensure shape detect result is correct, we detect twice with a few seconds gap to check if results are the same.

code arragemnet:

-    The code file for work4 is seperate from the main code for further improvement on the coding style.
-    Heavliy used simple task functions like rotation and signal (led and sound) have been seperated from the original file to increase simplicity and reusability.
-    Based on the experience we collected from demo4 and demo5, we carefully develop the map using view_nevigation package.- -    To improve the runtime performance, we choose to not launch rviz, this can be enabled through commenting out lines in launch file.

map and waypoint strategy:

-    After a fairly accurate map is established, we set the way points based on the map. By testing out each waypoint one by one, we want to make sure the run time error genreate by the odem has the minimum effect on the final parking spot.
-    Since the usb camera is stilling running during parking into these red squares, it is likely that the robot takes the parking red square as the functional red lines. New global varibies have set to avoid these conflicts.
searching strategy:
-    We used exhaustive search for the parking spot to make sure the robot complete the task and fit into all the squares.
-    Set initial pose when the robot is off ramp instead of the start point of the game to help localization and precision.
-    The docking process is based on waypoints. We test the waypoints one by one to ensure the the robot will dock on point.
-    The robot will skip the rest of waypoints if all task at location 4 have been completed.

#### Sources
- https://github.com/jackykc/comp5
- https://github.com/cmput412
- https://github.com/bofrim/CMPUT_412
- https://github.com/nwoeanhinnogaehr/412-W19-G5-public
- https://github.com/stwklu/CMPUT_412_code/
- https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
- https://github.com/TianqiCS/CMPUT-412-C2
- https://github.com/HumphreyLu6/CMPUT412_demo5_p2
- https://www.cnblogs.com/kuangxionghui/p/8335853.html
