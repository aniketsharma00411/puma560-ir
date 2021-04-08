<h1 align="center">Introduction to Robotics</h1>

<h3 align="center">Assignment</h3>

Submission for Assignment of Introduction to Robotics (BCCS-9402) course

Submitted by: Aniket Sharma (2019BCS-008)

Problem statement:
Write a program to calculate Forward Kinematics and Inverse Kinematics of a PUMA 560 robot.

<h4>Forward Kinematics</h4>

INPUT:

It takes the DH Table as input.

    1. (float) alpha0
    2. (float) a0
    3. (float) d1
    4. (float) theta1
    5. (float) alpha1
    6. (float) a1
    7. (float) d2
    8. (float) theta2
    9. (float) alpha2
    10. (float) a2
    11. (float) d3
    12. (float) theta3
    13. (float) alpha3
    14. (float) a3
    15. (float) d4
    16. (float) theta4
    17. (float) alpha4
    18. (float) a4
    19. (float) d5
    20. (float) theta5
    21. (float) alpha5
    22. (float) a5
    23. (float) d6
    24. (float) theta6

OUTPUT:

    1. (numpy array) End effector position

Example:

<img src="https://github.com/aniketsharma00411/puma560-ir/blob/main/example_forward_kinematics.png" alt="example_forward_kinematics.png">

<h4>Inverse Kinematics</h4>

INPUT:

    1. (float) a2
    2. (float) a3
    3. (float) d3
    4. (float) d4
    5. (4 lines of space separated float) Transformation Matrix

OUTPUT:

    1. (list of tuple of float) All possible configurations of joint angles

Example:

<img src="https://github.com/aniketsharma00411/puma560-ir/blob/main/example_inverse_kinematics.png" alt="example_inverse_kinematics.png">
