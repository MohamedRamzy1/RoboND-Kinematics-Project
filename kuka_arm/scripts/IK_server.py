#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters
        DH_Table = {alpha0: 0,     a0: 0,      d1: 0.75, 
             alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
             alpha2: 0,     a2: 1.25,   d3: 0,
             alpha3: -pi/2, a3: -0.054, d4: 1.5,
             alpha4: pi/2,  a4: 0,      d5: 0,
             alpha5: -pi/2, a5: 0,      d6: 0,
             alpha6: 0,     a6: 0,      d7: 0.303, q7: 0}
	# Define Modified DH Transformation matrix
        def homo_transform(alpha, a, d, q):
            h_t = Matrix([[cos(q),            -sin(q),           0,           a],
                        [sin(q)*cos(alpha), cos(alpha)*cos(q), -sin(alpha), -sin(alpha)*d],
                        [sin(alpha)*sin(q), sin(alpha)*cos(q), cos(alpha),  cos(alpha)*d],
                        [0,                 0,                 0,           1]])
            return h_t
	# Create individual transformation matrices
	#Each line contains a transformation from one link to the other
        T0_1 = homo_transform(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2 = homo_transform(alpha1, a1, d2, q2).subs(DH_Table)
        T2_3 = homo_transform(alpha2, a2, d3, q3).subs(DH_Table)
        T3_4 = homo_transform(alpha3, a3, d4, q4).subs(DH_Table)
        T4_5 = homo_transform(alpha4, a4, d5, q5).subs(DH_Table)
        T5_6 = homo_transform(alpha5, a5, d6, q6).subs(DH_Table)
        T6_E = homo_transform(alpha6, a6, d7, q7).subs(DH_Table)
	T0_3 = simplify(T0_1 * T1_2 * T2_3) #calculating the transform from link 0 to 3
	T0_3_inv = T0_3.inv() #calculating the inverse

	# Extract rotation matrices from the transformation matrices
        def rot_x(q):
            R_x = Matrix([[ 1,              0,        0],
                      [ 0,   cos(q),  -sin(q)],
                      [ 0,   sin(q),  cos(q)]])
            return R_x
        
        def rot_y(q):              
            R_y = Matrix([[ cos(q),   0,  sin(q)],
                      [      0,        1,       0],
                      [-sin(q),     0, cos(q)]])
            return R_y

        def rot_z(q):    
            R_z = Matrix([[ cos(q),  -sin(q),       0],
                      [ sin(q),   cos(q),       0],
                      [      0,        0,       1]])
            return R_z
        R_error = rot_z(pi) * rot_y(-pi/2) #Defining error correction matrix
        T0_E = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E) #the whole transform from link 0 to End effector
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    EE_pos = Matrix([[px],[py],[pz]])
            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_error
            WC = EE_pos - 0.303*Rrpy[:,2] #calculating WC from EE position and the rotation matrix
	    # Calculate joint angles using Geometric IK method
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            theta1 = atan2(WC[1], WC[0])
	    #calculating the intermediate angles a, b to compute theta 2 and theta 3
            dist_to_ground = ((WC[1]) **2 + WC[0]**2)**0.5
            c = ((dist_to_ground-DH_Table[a1])**2 + (WC[2]-DH_Table[d1])**2)**0.5
            a = (DH_Table[d4]**2 + DH_Table[a3]**2)**0.5
            b = DH_Table[a2]
            a_angle = acos((b**2 + c**2 - a**2)/(2*b*c))
	    theta2 = pi/2 - a_angle - atan2(WC[2]-DH_Table[d1], dist_to_ground-DH_Table[a1])
            b_angle = acos((a**2 + b**2 - c**2)/(2*a*b))
            theta3 = pi/2 - b_angle + atan2(DH_Table[a3], DH_Table[d4])
            #calculating theta4, 5 , 6.
            Rrpy = Rrpy.row_insert(3, Matrix([[0, 0, 0]]))
            Rrpy = Rrpy.col_insert(3, Matrix([[0], [0], [0], [1]]))
            R3_6 = (T0_3_inv * Rrpy).evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = acos(R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
