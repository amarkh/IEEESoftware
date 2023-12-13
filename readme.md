This repo will contain the IEEE Hawrdware team software code for the 2023/2024 school year Hardware competition.

The code will be at all levels:
  - Low level code currently for Arduinos that contain PID controllers for four DC motors
    - I plan on migrating the code to Raspberry Pi Picos as the device contains more interrupt pins and is smaller, easier to fit on our smaller robot.
  - Higher level code in python
    - As it stands, this code is able to detect lines in a given video feed and return the x,y,z coordinates of all the points on the line.
    - Will need to update the code to make it more accurate and only extract the needed data.
    - May migrate the code over to C++ for faster speeds of computation.
    - The code we are using takes advantage of the realsense library since our robot will primarily be using an Intel Realsense D435if
