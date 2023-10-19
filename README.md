You can watch the result at
https://www.youtube.com/watch?v=KDQSH_u8n48

You can see the project detail at: 
https://wordpress.com/post/evrid.wordpress.com/24

## Required Products
* MATLAB R2022a or later
* Image Processing Toolbox
* Computer Vision System Toolbox
* Statistics and Machine Learning Toolbox
* Robotics System Toolbox

%First download the folder, put the image we want process inside
there, then create a test.csv file (by create test.txt first then change test.txt to test.csv), then click open calculateFinalUpload.m
then we enter the origion (by go to the center of the paper then write down the x and y value) and ScaleValue and LiftPenHeight we want, 
then it ask us to ask us to select a file, we change file type to ALL FILES and select a
image located within the same folder of the MATLAB program, then the
output will go to test.csv

For Fanuc robot:
then open HandlingPro, right click on Targets, "import point data", select the csv we have
then right click "Target Groups", click "add target group", select the one we created then right click "edit target group"
then we import all targets, in "Motion Line Data", deselect both approach and retreat, then click generate TP file, then export the TP to our drive
then load to the robot arm.

Inspired by:
Using Targets in Roboguide to Visualize Path Planning of a FANUC Robot
https://sites.asee.org/edgd/wp-content/uploads/sites/22/2017/12/Part25-Li.pdf
Parts code from:
Tohru Kikawada (2022). Portrait Drawing using Computer Vision and Robot Manipulator (https://www.mathworks.com/matlabcentral/fileexchange/67926-portrait-drawing-using-computer-vision-and-robot-manipulator), MATLAB Central File Exchange. Retrieved June 20, 2022.

you can see the steps at
https://www.youtube.com/watch?v=k4gOBWY9oB4
