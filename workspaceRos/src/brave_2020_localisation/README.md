# brave_2020_localisation
Software to localise a robot (without GPS) using only cameras and known marks, without needing to identify them.
Initially designed for Brave sailboat from ENSTA Bretagne. Measures of heading and velocity highly recommanded.
Updated repository: https://github.com/Matthix7/brave_2020_localisation

## Dependencies
- Python 2.7
- ROS Melodic
- opencv 3.2.0
- pyibex
- vibes
- rospkg
- pyautogui
- utm
- itertools

## Motivation
Many localisation methods, while being highly efficient, rely on the use of landmarks that are supposed to be identifiable by the robot. This is often possible using various shapes, colors, codes...etc. But in the case of a boat in the ocean, often the marks are limited to buoys. And if you do not have many colors for your buoys, or if you want to have an algorithm that will not fail you because of a bad identification of your mark, another method seems preferable.  
As an alternative solution for localisation issues, we propose here to use the theory of intervals to delimit the areas where the sailboat/robot can possibly be given what it can see from its cameras. (see [this paper](https://www.ensta-bretagne.fr/jaulin/intervalcourse.pdf) for further information)  
The principle is simple to understand. Given an initial area for the position of the robot:
- either it cannot see a buoy, then at each time increment the area where the boat can possibly be is computed from the previous area accordingly with the heading and speed of the boat and the uncertainties coming from the model and the environment. This results in a bigger and shifted area.
- either it can see a buoy, then it is pretty close to it (depends on the image quality), oriented accordingly with the heading measure, and in an area that is compatible with its previous possible area. This results in a smaller area.

## What is needed
- The coordinates of the different landmarks
- A field of research, from which the robot cannot escape (can be wide)
- A way to get the headings of the landmarks in the robot's coordinate system
- A measure of the velocity of the robot, or at least an estimation of min & max.
- A measure of the heading of the robot (0-360 possible, but results will be much less precise)
- An evolution model for the robot (default: Dubin's car with drift) 

## Usage: interactive way
- Take a picture of the field of research and save it as base_map.png under `src/Localisation/base_map`.
- Use `roslaunch brave_2020_localisation real_test_bench.launch display:=true`.
- Your picture will pop up. You can double-click on it where you put each of your landmarks to give their coordinates.
- You can read `real_test_bench.launch` under `/launch` to get more information and fill in more of your robot's characteristics.
- Once you have validated the coordinates of your landmarks, the localisation algorith begins looping. It expects to headings of the marks, the heading of the robot and the speed of the robot respectively in topics "buoys_directions", "heading" and "speed". It outputs the areas where the robot can possibly be in topic "boat_possible_positions". Each of these areas is defined by its center (GPS coordinates), its length along WE axis and its length along NS axis. Feel free to read `src/Localisation/test_bench_intervals_ros.py` for a better understanding.  
Note: this usage is not recommended for live use in a robot, better for visualisation.

## Usage: coding way
- Modify directly `/launch/real_test_bench.launch`, and then use `roslaunch brave_2020_localisation real_test_bench.launch`.
- The algorithm will begin looping without display.
Note: a later update will allow to save the inputs of interactive usage for a mission without monitor.

## Example
Once you have correctly installed the package in a ROS workspace and installed the required *Python* libraries, you can run `roslaunch brave_2020_localisation example.launch` in your terminal. This will launch a simulation of a robot and the localisation software with its display.  
- When you run the command above, an *OpenCV* window should pop up with the *base_map* image. This image corresponds to the field of research that will be used for the localisation task. 
- If you have not changed the image, you should see two red dots marked "1" and "2". These are the locations of the *virtual* landmarks used for this simulation. Double-click on "1" and then "2" (lines should be printed in your terminal). This creates the link between these pixels and the GPS coordinates that appear in *example.launch*.
- Hit *Enter* twice to confirm.
- You are now entering the simulation. The landmarks are where you double-clicked before and the robot is the moving red dot (with a line that indicates its heading). You can control the moves of the robot **in your terminal** using the arrows of your keyboard (acceleration and rotation).  

 Initially, the only location knowledge that we have is that the robot is in the field (*ie* the image). While moving the robot fast enough or making it *see* a landmark (ie looking in the direction of the landmark and close enough to it), you will see one or several bright areas and the rest of the image quite dark. The bright areas are all the estimated possible positions of the robot computed by the localisation software (*ie* independant from the simulation). These positions are all the locations compatible with the moves and measures of the robot, given the parameters in the launchfile.

## Brave: controllers & low-level software
* https://github.com/QuentinCar/Brave
