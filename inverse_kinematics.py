"""
Submission for Assignment of Introduction to Robotics (BCCS-9402) course

Submitted by: Aniket Sharma (2019BCS-008)

Problem statement:
Write a program to calculate Inverse Kinematics of a PUMA 560 robot.

INPUT:

    1. (float) a2
    2. (float) a3
    3. (float) d3
    4. (float) d4
    5. (4 lines of space separated float) Transformation Matrix

OUTPUT:

    1. (list of tuple of float) All possible configurations of joint angles

Example:

Enter a2: 3
Enter a3: 3
Enter d3: 3
Enter d4: 3
Enter 4x4 Transformation Matrix:
0.23409161 -0.38798654 -0.89144129  3.05834348
-0.7910126  -0.60910276  0.05738376  3.58554831
-0.56524347  0.69170823 -0.44948808 -4.95464258
0 0 0 1


Possible joint angles are:
theta1: 10.000000039510034, theta2: 15.000000056927114, theta3: 19.99999993476635, theta4: 24.999999351160376, theta5: 28.80078849071206, theta6: 35.00589773743219

theta1: -90.92596621867615, theta2: 29.336784662814388, theta3: 19.99999993476635, theta4: -109.32849858479103, theta5: 70.98859921170474, theta6: 104.00057873583638

theta1: 10.000000039510034, theta2: 82.30199975212324, theta3: -109.99999993476636, theta4: 12.199265056593227, theta5: 90.31569452938403, theta6: 57.057980819827115

theta1: -90.92596621867615, theta2: 150.66321533718562, theta3: -109.99999993476636, theta4: -106.26353314682007, theta5: 68.32965341027372, theta6: 95.19353205783585

theta1: 10.000000039510034, theta2: 15.000000056927114, theta3: 19.99999993476635, theta4: 204.99999935116037, theta5: -28.80078849071206, theta6: 215.00589773743218

theta1: -90.92596621867615, theta2: 29.336784662814388, theta3: 19.99999993476635, theta4: 70.67150141520897, theta5: -70.98859921170474, theta6: 284.00057873583637

theta1: 10.000000039510034, theta2: 82.30199975212324, theta3: -109.99999993476636, theta4: 192.19926505659322, theta5: -90.31569452938403, theta6: 237.05798081982712

theta1: -90.92596621867615, theta2: 150.66321533718562, theta3: -109.99999993476636, theta4: 73.73646685317993, theta5: -68.32965341027372, theta6: 275.19353205783585
"""

import math
import numpy as np


def input_robot_features():
    """
    Method to take input of robot features from user.

    INPUT:

    1. (float) a2
    2. (float) a3
    3. (float) d3
    4. (float) d4

    OUTPUT:

    1. (tuple of float) Robot features
    """

    return (float(input('Enter a2: ')), float(input('Enter a3: ')), float(input('Enter d3: ')), float(input('Enter d4: ')))


def input_transformation_matrix():
    """
    Method to take input of Transformation Matrix from user.

    INPUT:

    1. (4 lines of space separated float) Transformation Matrix

    OUTPUT:

    1. (numpy array) Transformation Matrix
    """

    T = np.zeros((4, 4))
    print('Enter 4x4 Transformation Matrix:')
    for i in range(4):
        T[i] = np.array(list(map(float, input().split())))

    return T


def inverse_kinematics(T, a2, a3, d3, d4):
    """
    Method to calculate joint angles.

    INPUT:

    1. (numpy array) Transformation Matrix
    2. (float) a2
    3. (float) a3
    4. (float) d3
    5. (float) d4

    OUTPUT:

    1. (list of tuple of float) All possible configurations of joint angles
    """

    px = T[0, 3]
    py = T[1, 3]
    pz = T[2, 3]

    # calculating theta1
    theta1 = []
    theta1.append(math.degrees(math.atan2(py, px) -
                               math.atan2(d3, (px**2+py**2-d3**2)**0.5)))
    theta1.append(math.degrees(math.atan2(py, px) -
                               math.atan2(d3, -((px**2+py**2-d3**2)**0.5))))

    K = (px**2+py**2+pz**2-a2**2-a3**2-d3**2-d4**2)/(2*a2)

    # calculating theta3
    theta3 = []
    theta3.append(math.degrees(math.atan2(a3, d4) -
                               math.atan2(K, (a3**2+d4**2-K**2)**0.5)))
    theta3.append(math.degrees(math.atan2(a3, d4) -
                               math.atan2(K, -((a3**2+d4**2-K**2)**0.5))))

    # calculating theta2
    theta2 = []

    s1 = math.sin(math.radians(theta1[0]))
    c1 = math.cos(math.radians(theta1[0]))
    s3 = math.sin(math.radians(theta3[0]))
    c3 = math.cos(math.radians(theta3[0]))
    theta23 = math.degrees(math.atan2(
        ((-a3-(a2*c3))*pz)-(((c1*px)+(s1*py))*(d4-(a2*s3))), (((a2*s3)-d4)*pz)+((a3+(a2*c3))*((c1*px)+(s1*py)))))
    theta2.append(theta23-theta3[0])
    s1 = math.sin(math.radians(theta1[1]))
    c1 = math.cos(math.radians(theta1[1]))
    s3 = math.sin(math.radians(theta3[0]))
    c3 = math.cos(math.radians(theta3[0]))
    theta23 = math.degrees(math.atan2(
        ((-a3-a2*c3)*pz)-((c1*px+s1*py)*(d4-a2*s3)), ((a2*s3-d4)*pz)-((a3+a2*c3)*(c1*px+s1*py))))
    theta2.append(theta23-theta3[0])
    s1 = math.sin(math.radians(theta1[0]))
    c1 = math.cos(math.radians(theta1[0]))
    s3 = math.sin(math.radians(theta3[1]))
    c3 = math.cos(math.radians(theta3[1]))
    theta23 = math.degrees(math.atan2(
        ((-a3-a2*c3)*pz)-((c1*px+s1*py)*(d4-a2*s3)), ((a2*s3-d4)*pz)-((a3+a2*c3)*(c1*px+s1*py))))
    theta2.append(theta23-theta3[1])
    s1 = math.sin(math.radians(theta1[1]))
    c1 = math.cos(math.radians(theta1[1]))
    s3 = math.sin(math.radians(theta3[1]))
    c3 = math.cos(math.radians(theta3[1]))
    theta23 = math.degrees(math.atan2(
        ((-a3-a2*c3)*pz)-((c1*px+s1*py)*(d4-a2*s3)), ((a2*s3-d4)*pz)-((a3+a2*c3)*(c1*px+s1*py))))
    theta2.append(theta23-theta3[1])

    r13 = T[0, 2]
    r23 = T[1, 2]
    r33 = T[2, 2]

    # calculating theta4
    theta4 = []

    s1 = math.sin(math.radians(theta1[0]))
    c1 = math.cos(math.radians(theta1[0]))
    s23 = math.sin(math.radians(theta2[0]+theta3[0]))
    c23 = math.cos(math.radians(theta2[0]+theta3[0]))
    theta4.append(math.degrees(math.atan2((-r13*s1)+(r23*c1),
                                          (-r13*c1*c23)-(r23*s1*c23)+(r33*s23))))
    s1 = math.sin(math.radians(theta1[1]))
    c1 = math.cos(math.radians(theta1[1]))
    s23 = math.sin(math.radians(theta2[1]+theta3[0]))
    c23 = math.cos(math.radians(theta2[1]+theta3[0]))
    theta4.append(math.degrees(math.atan2((-r13*s1)+(r23*c1),
                                          (-r13*c1*c23)-(r23*s1*c23)+(r33*s23))))
    s1 = math.sin(math.radians(theta1[0]))
    c1 = math.cos(math.radians(theta1[0]))
    s23 = math.sin(math.radians(theta2[2]+theta3[1]))
    c23 = math.cos(math.radians(theta2[2]+theta3[1]))
    theta4.append(math.degrees(math.atan2((-r13*s1)+(r23*c1),
                                          (-r13*c1*c23)-(r23*s1*c23)+(r33*s23))))
    s1 = math.sin(math.radians(theta1[1]))
    c1 = math.cos(math.radians(theta1[1]))
    s23 = math.sin(math.radians(theta2[3]+theta3[1]))
    c23 = math.cos(math.radians(theta2[3]+theta3[1]))
    theta4.append(math.degrees(math.atan2((-r13*s1)+(r23*c1),
                                          (-r13*c1*c23)-(r23*s1*c23)+(r33*s23))))

    # calculating theta5
    theta5 = []

    s1 = math.sin(math.radians(theta1[0]))
    c1 = math.cos(math.radians(theta1[0]))
    s23 = math.sin(math.radians(theta2[0]+theta3[0]))
    c23 = math.cos(math.radians(theta2[0]+theta3[0]))
    s4 = math.sin(math.radians(theta4[0]))
    c4 = math.cos(math.radians(theta4[0]))
    theta5.append(math.degrees(math.atan2((-r13*(c1*c23*c4+s1*s4)) -
                                          (r23*(s1*c23*c4))+(r33*s23*c4), (-r13*c1*s23)-(r23*s1*s23)-(r33*c23))))

    s1 = math.sin(math.radians(theta1[1]))
    c1 = math.cos(math.radians(theta1[1]))
    s23 = math.sin(math.radians(theta2[1]+theta3[0]))
    c23 = math.cos(math.radians(theta2[1]+theta3[0]))
    s4 = math.sin(math.radians(theta4[1]))
    c4 = math.cos(math.radians(theta4[1]))
    theta5.append(math.degrees(math.atan2((-r13*(c1*c23*c4+s1*s4)) -
                                          (r23*(s1*c23*c4))+(r33*s23*c4), (-r13*c1*s23)-(r23*s1*s23)-(r33*c23))))

    s1 = math.sin(math.radians(theta1[0]))
    c1 = math.cos(math.radians(theta1[0]))
    s23 = math.sin(math.radians(theta2[2]+theta3[1]))
    c23 = math.cos(math.radians(theta2[2]+theta3[1]))
    s4 = math.sin(math.radians(theta4[2]))
    c4 = math.cos(math.radians(theta4[2]))
    theta5.append(math.degrees(math.atan2((-r13*(c1*c23*c4+s1*s4)) -
                                          (r23*(s1*c23*c4))+(r33*s23*c4), (-r13*c1*s23)-(r23*s1*s23)-(r33*c23))))

    s1 = math.sin(math.radians(theta1[1]))
    c1 = math.cos(math.radians(theta1[1]))
    s23 = math.sin(math.radians(theta2[3]+theta3[1]))
    c23 = math.cos(math.radians(theta2[3]+theta3[1]))
    s4 = math.sin(math.radians(theta4[3]))
    c4 = math.cos(math.radians(theta4[3]))
    theta5.append(math.degrees(math.atan2((-r13*(c1*c23*c4+s1*s4)) -
                                          (r23*(s1*c23*c4))+(r33*s23*c4), (-r13*c1*s23)-(r23*s1*s23)-(r33*c23))))

    r11 = T[0, 0]
    r21 = T[1, 0]
    r31 = T[2, 0]

    # calculating theta6
    theta6 = []

    s1 = math.sin(math.radians(theta1[0]))
    c1 = math.cos(math.radians(theta1[0]))
    s23 = math.sin(math.radians(theta2[0]+theta3[0]))
    c23 = math.cos(math.radians(theta2[0]+theta3[0]))
    s4 = math.sin(math.radians(theta4[0]))
    c4 = math.cos(math.radians(theta4[0]))
    s5 = math.sin(math.radians(theta5[0]))
    c5 = math.cos(math.radians(theta5[0]))
    theta6.append(math.degrees(math.atan2(
        (-r11*(c1*c23*s4-s1*c4)-(r21*(s1*c23*s4+c1*c4))+r31*s23*s4), (r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5))+(r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5))-(r31*(s23*c4*c5+c23*s5)))))
    s1 = math.sin(math.radians(theta1[1]))
    c1 = math.cos(math.radians(theta1[1]))
    s23 = math.sin(math.radians(theta2[1]+theta3[0]))
    c23 = math.cos(math.radians(theta2[1]+theta3[0]))
    s4 = math.sin(math.radians(theta4[1]))
    c4 = math.cos(math.radians(theta4[1]))
    s5 = math.sin(math.radians(theta5[1]))
    c5 = math.cos(math.radians(theta5[1]))
    theta6.append(math.degrees(math.atan2(
        (-r11*(c1*c23*s4-s1*c4)-(r21*(s1*c23*s4+c1*c4))+r31*s23*s4), (r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5))+(r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5))-(r31*(s23*c4*c5+c23*s5)))))
    s1 = math.sin(math.radians(theta1[0]))
    c1 = math.cos(math.radians(theta1[0]))
    s23 = math.sin(math.radians(theta2[2]+theta3[1]))
    c23 = math.cos(math.radians(theta2[2]+theta3[1]))
    s4 = math.sin(math.radians(theta4[2]))
    c4 = math.cos(math.radians(theta4[2]))
    s5 = math.sin(math.radians(theta5[2]))
    c5 = math.cos(math.radians(theta5[2]))
    theta6.append(math.degrees(math.atan2(
        (-r11*(c1*c23*s4-s1*c4)-(r21*(s1*c23*s4+c1*c4))+r31*s23*s4), (r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5))+(r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5))-(r31*(s23*c4*c5+c23*s5)))))
    s1 = math.sin(math.radians(theta1[1]))
    c1 = math.cos(math.radians(theta1[1]))
    s23 = math.sin(math.radians(theta2[3]+theta3[1]))
    c23 = math.cos(math.radians(theta2[3]+theta3[1]))
    s4 = math.sin(math.radians(theta4[3]))
    c4 = math.cos(math.radians(theta4[3]))
    s5 = math.sin(math.radians(theta5[3]))
    c5 = math.cos(math.radians(theta5[3]))
    theta6.append(math.degrees(math.atan2(
        (-r11*(c1*c23*s4-s1*c4)-(r21*(s1*c23*s4+c1*c4))+r31*s23*s4), (r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5))+(r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5))-(r31*(s23*c4*c5+c23*s5)))))

    # creating a list of joint angles calculated above
    joint_angles = []

    joint_angles.append(
        (theta1[0], theta2[0], theta3[0], theta4[0], theta5[0], theta6[0]))
    joint_angles.append(
        (theta1[1], theta2[1], theta3[0], theta4[1], theta5[1], theta6[1]))
    joint_angles.append(
        (theta1[0], theta2[2], theta3[1], theta4[2], theta5[2], theta6[2]))
    joint_angles.append(
        (theta1[1], theta2[3], theta3[1], theta4[3], theta5[3], theta6[3]))
    joint_angles.append(
        (theta1[0], theta2[0], theta3[0], theta4[0]+180, -theta5[0], theta6[0]+180))
    joint_angles.append(
        (theta1[1], theta2[1], theta3[0], theta4[1]+180, -theta5[1], theta6[1]+180))
    joint_angles.append(
        (theta1[0], theta2[2], theta3[1], theta4[2]+180, -theta5[2], theta6[2]+180))
    joint_angles.append(
        (theta1[1], theta2[3], theta3[1], theta4[3]+180, -theta5[3], theta6[3]+180))

    return joint_angles


def print_solution(joint_angles):
    """
    Method to print the answer.

    INPUT:

    1. (list of tuple of float) All possible configurations of joint angles
    """

    print('\n\nPossible joint angles are:')
    for i in range(8):
        print('theta1: {}, theta2: {}, theta3: {}, theta4: {}, theta5: {}, theta6: {}'.format(
            joint_angles[i][0], joint_angles[i][1], joint_angles[i][2], joint_angles[i][3], joint_angles[i][4], joint_angles[i][5]))
        print()


if __name__ == '__main__':
    a2, a3, d3, d4 = input_robot_features()
    T = input_transformation_matrix()
    joint_angles = inverse_kinematics(T, a2, a3, d3, d4)

    print_solution(joint_angles)
