"""
Submission for Assignment of Introduction to Robotics (BCCS-9402) course

Submitted by: Aniket Sharma (2019BCS-008)

Problem statement:
Write a program to calculate Forward Kinematics of a PUMA 560 robot.

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

alpha0: 0
a0: 0
d1: 0
theta1: 10

alpha1: -90
a1: 0
d2: 0
theta2: 15

alpha2: 0
a2: 3
d3: 3
theta3: 20

alpha3: -90
a3: 3
d4: 3
theta4: 25

alpha4: 90
a4: 0
d5: 0
theta5: 30

alpha5: -90
a5: 0
d6: 0
theta6: 35


End effector position: (3.0583434826357276, 3.585548307919805, -4.954642577227676)
"""

import math
import numpy as np


def input_dh_table(num_joints):
    """
    Method to take input from user.

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

    1. (numpy array) DH Table
    """

    dh_table = np.zeros((num_joints, 4))
    for i in range(num_joints):
        parameters = np.array([float(input('alpha{}: '.format(i))), float(input('a{}: '.format(
            i))), float(input('d{}: '.format(i+1))), float(input('theta{}: '.format(i+1)))])
        dh_table[i] = parameters
        print()

    return dh_table


def T(dh_table, i):
    """
    Method to calculate Transformation Matrix for axis i to i+1.

    INPUT:

    1. (numpy array) DH Table
    2. (int) axis

    OUTPUT:

    1. (numpy array) Transformation Matrix
    """

    a_i1 = dh_table[i][1]
    alpha_i1 = math.radians(dh_table[i][0])
    d_i = dh_table[i][2]
    theta_i = math.radians(dh_table[i][3])

    return np.array([[math.cos(theta_i), -math.sin(theta_i), 0, a_i1],
                     [math.cos(alpha_i1)*math.sin(theta_i), math.cos(alpha_i1) *
                      math.cos(theta_i), -math.sin(alpha_i1), -d_i*math.sin(alpha_i1)],
                     [math.sin(alpha_i1)*math.sin(theta_i), math.sin(alpha_i1) *
                      math.cos(theta_i), math.cos(alpha_i1), d_i*math.cos(alpha_i1)],
                     [0, 0, 0, 1]])


def final_transformation_matrix(dh_table):
    """
    Method to calculate the final Transformation Matrix.

    INPUT:

    1. (numpy array) DH Table

    OUTPUT:

    1. (numpy array) Final Transformation Matrix
    """

    T_final = T(dh_table, 0)
    for i in range(1, dh_table.shape[0]):
        T_final = T_final @ T(dh_table, i)

    return T_final


def print_solution(end_effector_position):
    """
    Method to print the answer.

    INPUT:

    1. (numpy array) End effector position
    """

    print('\nEnd effector position: ({}, {}, {})'.format(
        end_effector_position[0], end_effector_position[1], end_effector_position[2]))


if __name__ == '__main__':
    num_joints = 6
    dh_table = input_dh_table(num_joints)
    T_final = final_transformation_matrix(dh_table)

    end_effector_position = T_final @ np.array([0, 0, 0, 1])

    print_solution(end_effector_position)
