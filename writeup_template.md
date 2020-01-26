## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/kinematic_analysis.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/correct_1.png
[image4]: ./misc_images/correct_2.png
[image6]: ./misc_images/thetas1.png
[image7]: ./misc_images/thetas2.png
[image8]: ./misc_images/thetas.jpg
[image9]: ./misc_images/wc.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]

By drawing the joints of the kuka KR210 and selecting the axes of each joint, Also by analysing the **kr210.urdf.xacro** file to get the appropriate values of the joints we can obtain the DH parameter table as follows

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
1   | 0 |	0 	|0.75 	|theta_1
2   |	-pi/2 |	0.35 |	0 	|theta_2 - pi/2
3 	|0 |	1.25 |	0 |	theta_3
4 	|-pi/2 	|-0.054 	|1.5 |	theta_4
5 	|pi/2 	|0 |	0 	|theta_5
6 	|-pi/2 	|0 	|0 	|theta_6
7 	|0 	|0 	|0.303 	|theta_7

Now let's go into this part in the code step by step:

We firstly define our symbols then we define our DH_Table


		q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
		d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
		a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
		alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
		DH_Table = {alpha0: 0,     a0: 0,      d1: 0.75, 
		     alpha1: -pi/2, a1: 0.35,   d2: 0,     q2: q2-pi/2,
		     alpha2: 0,     a2: 1.25,   d3: 0,
		     alpha3: -pi/2, a3: -0.054, d4: 1.5,
		     alpha4: pi/2,  a4: 0,      d5: 0,
		     alpha5: -pi/2, a5: 0,      d6: 0,
		     alpha6: 0,     a6: 0,      d7: 0.303, q7: 0}




#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The next step is defining the transformation matrices between each link and the other to finally obtain a homogenious translation matrix between the first link and the end-effector

We Define our homogenous transform as follows:

        def homo_transform(alpha, a, d, q):
            h_t = Matrix([[cos(q),            -sin(q),           0,           a],
                        [sin(q)*cos(alpha), cos(alpha)*cos(q), -sin(alpha), -sin(alpha)*d],
                        [sin(alpha)*sin(q), sin(alpha)*cos(q), cos(alpha),  cos(alpha)*d],
                        [0,                 0,                 0,           1]])
            return h_t
            
Then we compute the matrix between each link and the other as follows, We also define the transform matrix between link 0 and 3 and compute its inverse as we will use it in IK calculations.

        T0_1 = homo_transform(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2 = homo_transform(alpha1, a1, d2, q2).subs(DH_Table)
        T2_3 = homo_transform(alpha2, a2, d3, q3).subs(DH_Table)
        T3_4 = homo_transform(alpha3, a3, d4, q4).subs(DH_Table)
        T4_5 = homo_transform(alpha4, a4, d5, q5).subs(DH_Table)
        T5_6 = homo_transform(alpha5, a5, d6, q6).subs(DH_Table)
        T6_E = homo_transform(alpha6, a6, d7, q7).subs(DH_Table)
		T0_3 = simplify(T0_1 * T1_2 * T2_3)
		T0_3_inv = T0_3.inv()
		
then we define rotation matrices around x , y , z axes to calculate the transformation matrix between the first link to the end effector, Also we define R_error matrix to correct the orientation of the end effector.

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
        R_error = rot_z(pi) * rot_y(-pi/2)
        T0_E = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_E)




#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Now let's go through deriving of the equations for theta angles step by step:

For theta one it is pretty straight forward that we use the arctan or the WC components, And for theta 2 and theta 3 we use trigeometry to derive them using intermediate angles a, b

![alt text][image8]

We first calculate WC from End effector position as follows:

![alt text][image9]

        EE_pos = Matrix([[px],[py],[pz]])
        Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_error
        WC = EE_pos - 0.303*Rrpy[:,2]
        
        
 Then we calculate the angles like this 
 
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


And for theta 4,5,6 we use Rrpy and the inverse ot the homogenous transform from link 0 to 3 and euler angles to derive the formula for them as follows:




![alt text][image6]
![alt text][image7]

And here is there implementation, We substitute with q1,q2,q3 in the inverted homogenious transform from link 0 to 3, And it is multiplied by Rrpy to correct the orientation.

        Rrpy = Rrpy.row_insert(3, Matrix([[0, 0, 0]]))
        Rrpy = Rrpy.col_insert(3, Matrix([[0], [0], [0], [1]]))
        R3_6 = (T0_3_inv * Rrpy).evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta5 = acos(R3_6[1,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])
        
        
 And here we are done we then provide thetas for the trajectory of the robot to perform pick and place operation



![alt text][image2]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 



I have implemented it with PyThon using libraries such as Sympy, The simulation is done on Gazebo and ROS.

I tried to optimize the code as far as i can, But to gain better performance we can work on optimizing the desicion maker and the path planner, As there is more optimum paths to reach the goal, So i think this is the bottleneck of the optimization, As i can see the robot reaches the goal successfully but it can do it much faster.



Here are screenshots for a correct pick and place operation.

![alt text][image3]
![alt text][image4]


