# autonomous-path-planning

PATH PLANNING BOT :-

ABSTRACT - Automatic path planning is one of the most challenging problems confronted by autonomous robots. Generating optimal paths for autonomous robots are some of the heavily studied subjects in mobile robotics applications. This paper documents the implementation of a path planning project using a mobile robot in a structured environment. The environment is detected through a camera and then a roadmap of the environment is built using some algorithms. Finally a graph search algorithm called A* is implemented that searches through the roadmap and finds an optimal path for robot to move from start position to goal position avoiding obstacles.

INTRODUCTION - Path planning can be taken as a task in which the robot, whether it is a robotic arm or mobile robot, has to navigate from its start point to a specific (destination or goal) point by avoiding collisions with the obstacles in the way. Path planning has widespread usage in mobile robotics, manufacturing and automation etc. This paper aims at the implementation of a path planning project in a structured environment using a small mobile robot. The environment consists of a thermocol sheets containing some obstacles. 
Colour papers have been used to identify the, obstacles and boundary of environment. Green colour indicates the boundaries, red colour indicates the obstacles and the robot is localized using April-tag. To detect the environment, a video surveillance camera (Logitech C270) is mounted on the top of this environment. In the environment a goal point will be set by user and the robot will have to reach this goal   point from its current position (start point) by following the shortest collision free path in the environment. In order to achieve this, the project can be divided into following main tasks.

i)   Detection of the environment through camera and cropping out the region of interest 
     First, an image is obtained through camera. Pixel value of corners of the environment is detected and region of interest is cropped using ROI function.
ii)  Detection of the obstacles in the environment. 
     The next step is to detect obstacles in the image which will be used along with the start and goal point to build a roadmap of the environment.
iii) Converting the image into a 7X7 grid. 
     The image is converted into an nXn grid such that the obstacle properly fits in the small block as shown in Fig 2.
iv)  Impleme­­­­ntation of a shortest path algorithm. 
     Then, a shortest path algorithm called A star (A*), will be implemented which will find optimal path by searching through all the paths provided by
     visibility graph. 
v)   Programming the robot to follow the optimal path from start position to goal position.  

METHODOLOGY - This section will describe the methodologies adapted for performing each of the tasks discussed in previous section.

1. Image Segmentation
  
In the field of robotics, the challenge is to do reliable segmentation of the scenes in order to plan the robot movement to perform a specific task. When the task is to identify few objects based on their color properties, threshold techniques have been used. In current case, as objects can be identified through their color, a thresholding technique has been used. Now, the image obtained from the camera is an RGB image. And region of interest (roi) is applied to exclude the surrounding portion which is not useful. It is then converted from RGB space to HSV space. To select the threshold values for the three colors (green, blue and red), any homogeneous part of the image that only contains the required color is selected and the maximum and minimum values of the hue, intensity and saturation are obtained from this region. These values serve as the threshold values. After selecting the threshold values, all the image pixels are checked and if HSV values of a pixel fall within the threshold values of any of the three color classes, that pixel is assigned a specific value and thus pixel is represented by a unique color in the output image.



2. Identification of the Boundary Points and obstacles 

In Fig. given below there are red points in the corners. The red points in corners are used to know the boundary of environment. These are obtained by calculating the two opposite centroids of the boundary regions (in blue color). Also the obstacles having red colour are thresholded. 

3. Conversion of image into a grid 

The image obtained after application of ROI is further converted into a 7x7 grid. Then the red obstacles covering particular grids are given a value 1 and rest of the grids are given a value 0. Now the matrix generated is given as input to A star algorithm to calculate shortest possible path. 	 

4. Study and application of Astar algorithm to generate shortest path.

A* Search algorithm is one of the best and popular technique used in path-finding and graph traversals.
A* Search algorithms, unlike other traversal techniques, it has “brains”. What it means is that it is really a smart algorithm which separates it from the other conventional algorithms. And it is also worth mentioning that many games and web-based maps use this algorithm to find the shortest path very efficiently (approximation).
Explanation -
Consider a square grid having many obstacles and we are given a starting cell and a target cell. We want to reach the target cell (if possible) from the starting cell as quickly as possible. Here A* Search Algorithm comes to the rescue.
What A* Search Algorithm does is that at each step it picks the node according to a value-‘f’ which is a parameter equal to the sum of two other parameters – ‘g’ and ‘h’. At each step it picks the node/cell having the lowest ‘f’, and process that node/cell.
We define ‘g’ and ‘h’ as simply as possible below
g = the movement cost to move from the starting point to a given square on the grid, following the path generated to get there.
h = the estimated movement cost to move from that given square on the grid to the final destination. This is often referred to as the heuristic, which is nothing but a kind of smart guess. We really don’t know the actual distance until we find the path, because all sorts of things can be in the way (walls, water, etc.).

5. Building of a robot having required communication setup.

Here we have used Bluetooth serial communication. The data is sent through Bluetooth module to Arduino from PC.
BLUETOOTH SERIAL COMMUNICATION IN UBUNTU 

i.) Setting up bluetooth module HC-05 
    Before being able to use the modules they need to be configured. A breadboard and some wires would be extremely helpful! We have used an Arduino loaded 
    with a sketch that simply reads from one serial port and transmits this data to the other serial port. Here’s the sketch:

/////////////////////////////////////////////////////////////////////////////////////////////////

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX
void setup() {
Serial.begin(9600);
pinMode(9,OUTPUT); digitalWrite(9,HIGH);
Serial.println("Enter AT commands:");
mySerial.begin(38400);
}
void loop()
{
if (mySerial.available())
Serial.write(mySerial.read());
if (Serial.available())
mySerial.write(Serial.read());
}

/////////////////////////////////////////////////////////////////////////////////////////////////

ii.) Connect the bluetooth module to the Arduino as follows:
   GND      ------   GND
    3.3      do not connect
    5.0      ------    5V
    RXD      ------    D11
    TXD      ------    D10
    KEY      do not connect
  STATE ------- pin 9
To enter AT command mode, remove 5v pin and press the reset button on the module while inserting 5v pin.
AT commands – 

AT COMMANDS
EXPLANATION
AT+NAME    
get/set name
AT+ADDR?   
get address of module (xxxx:xx:xxxxxx). This is the MAC address, write it down
AT+UART  
get/set communication parameters in format: baudrate, stopbits, parity. 
 Set to 9600,1,0 with AT+UART=9600,1,0

AT+ROLE    
get/set role. 0=SLAVE, 1=MASTER. Set to SLAVE

iii.) Once Bluetooth module is configured, follow these steps to communicate serially via Bluetooth
      Install BlueZ – Enter the following command to install the packages.
   		sudo apt-get install bluetooth bluez-utils
Scan for available  Bluetooth devices
hcitool scan
Copy the MAC id of the module
Copy the rfcomm.conf file to rfcommoriginal.conf so that if things do not work out one can always revert back
sudo cp /etc/bluetooth/rfcomm.conf rfcomm.conf.orig
Edit the rfcomm.conf file using the following command
sudo nano /etc/bluetooth/rfcomm.conf
	Make the changes in the file
#
# RFCOMM configuration file.
#
rfcomm0 {
 # Automatically bind the device at startup
 bind yes;
# Bluetooth address of the device
 device 98:D3:31:b0:80:6C;
# RFCOMM channel for the connection
 channel 1;
# Description of the connection
 comment "LS-ONE";
}
sudo echo "98:D3:31:B0:80:6C 1234" >> /var/lib/bluetooth/xx:xx:xx:xx:xx:xx/pincodes
Restart Bluetooth using the following command
sudo /etc/init.d/bluetooth restart
 As a first test we are going to echo something to the rfcomm0-device.
echo "TEST" > /dev/rfcomm0
Open serial monitor in Arduino to see output


6. Localization of robot using Apriltag.

AprilTag is a visual fiducial system, useful for a wide variety of tasks including augmented reality, robotics, and camera calibration. Targets can be created from an ordinary printer, and the AprilTag detection software computes the precise 3D position, orientation, and identity of the tags relative to the camera. The AprilTag library is implemented in C with no external dependencies. It is designed to be easily included in other applications, as well as be portable to embedded devices. Real-time performance can be achieved even on cell-phone grade processors. The following are different types of apriltags. We have used Tag36h11.


Future Work 
In this project, static environment is assumed i.e. the obstacles are fixed. However, when there are moving obstacles in the environment, it is required to re plan the path after specific intervals in order to know if any change occurred in the environment. D* [8] algorithm which is an extension of A*, can be implemented in this case.

Reference

1. OpenCV tutorials : 
http://opencv-srf.blogspot.in/
http://docs.opencv.org/2.4/modules/refman.html
2. STACKOVERFLOW :
 https://stackoverflow.com/questions/tagged/opencv
3. A* Algorithm
http://www.geeksforgeeks.org/a-search-algorithm/
4. Serial communication:
https://www.cmrr.umn.edu/~strupp/serial.html#2_5
https://salilkapur.wordpress.com/2013/03/08/communicating-with-arduino-using-c/
5. Bluetooth Serial Communication:
https://myraspberryandme.wordpress.com/2013/11/20/bluetooth-serial-communication-with-hc-05/
6. Apriltag
http://people.csail.mit.edu/kaess/apriltags/

*- Apriltag library should be installed.
*- Arduino interface should be installed.

	
	
	
