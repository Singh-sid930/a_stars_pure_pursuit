#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 

#############
# CONSTANTS #
#############

LOOKAHEAD_DISTANCE = 3 # meters
VELOCITY = 0.2 # m/s


###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points_x = [float(point[0]) for point in path_points]
path_points_y = [float(point[1]) for point in path_points]
path_points_w = [float(point[2]) for point in path_points]
path_points = (path_points_x,path_points_y,path_points_w)
path_points=np.array(path_points)

x_dif = np.zeros(len(path_points_x))
y_dif = np.zeros(len(path_points_y))
dist_arr = np.zeros(len(path_points_y))
goal=0



        
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


#############
# FUNCTIONS #
#############
    
# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Input data is PoseStamped message from topic /pf/viz/inferred_pose.
# Runs pure pursuit and publishes velocity and steering angle.
def callback(data):

    global goal

    global path_points_x
    global path_points_y
    global path_points_w

    qx=data.pose.orientation.x
    qy=data.pose.orientation.y
    qz=data.pose.orientation.z
    qw=data.pose.orientation.w

    quaternion = (qx,qy,qz,qw)
    euler = euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw =euler[2] 
    yaw= - yaw

    x=data.pose.position.x
    y=data.pose.position.y

    path_points_x = np.array(path_points_x)
    path_points_y=np.array(path_points_y)




    x_dif = path_points_x - x
    y_dif = path_points_y - y


    ## finding the distance of each way point from the current position 


    for i in range(len(x_dif)):
        dist_arr[i] = dist((path_points_x[i],path_points_y[i]),(x,y))


    ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)

    goal_arr = np.where(dist_arr<LOOKAHEAD_DISTANCE)[0] 


    ##finding the goal point which is the last in the set of points less than the lookahead distance
    ##if the closest points array could not be formed, then the point which is closest to the current position is the goal. 
    

    if(len(goal_arr)>0):
        #print(goal_arr)
        goal = goal_arr[len(goal_arr)-1] +1
        #print (goal)
    else: 
        goal = goal+1
        #print(goal) 


    ##finding the distance of the goal point from the vehicle coordinatesr


    L = dist_arr[goal]

    ##Transforming the goal point into the vehicle coordinate frame 

    xx_trans=path_points_x[goal]* math.cos(yaw)
    xy_trans=-path_points_y[goal]* math.sin(yaw)

    yx_trans=path_points_x[goal]* math.sin(yaw)
    yy_trans=path_points_y[goal]* math.cos(yaw)


    goal_x_veh_coord = xx_trans + xy_trans - x
    goal_y_veh_coord = yx_trans + yy_trans - y 




    
    angle_i = (2*goal_y_veh_coord)/L**2

    print(path_points_x[goal],path_points_y[goal],path_points_w[goal])
    print(x,y,180*yaw/math.pi)
    print(goal_y_veh_coord,angle_i)
    print("*******")



    



    # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

    # 1. Determine the current location of the vehicle (we are subscribed to vesc/odom)
    # Hint: Read up on PoseStamped message type in ROS to determine how to extract x, y, and yaw.
    

        

    # 2. Find the pat point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.



    # 3. Transform the goal point to vehicle coordinates. 
    
    

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle by the vehicle on board controller.
    # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.


    angle = angle_i
    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = angle

    
    pub.publish(msg)
    
    
if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, callback, queue_size=1)
    rospy.spin()

