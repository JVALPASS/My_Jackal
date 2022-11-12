#! /usr/bin/env python
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import tf
import alvinxy.alvinxy as axy
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
# Import geonav tranformation module
import geonav_transform.geonav_conversions as gc
reload(gc)
# Import AlvinXY transformation module
reload(axy)
# We create some constants with the corresponing vaules from the SimpleGoalState class

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4
# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received


def feedback_callback(feedback):
    rospy.loginfo("")


def get_xy_based_on_lat_long(p):
    # Define a local orgin, latitude and longitude in decimal degrees
    # GPS Origin
    olat = 49.9
    olon = 8.9

    xg2, yg2 = gc.ll2xy(p.lat, p.lon, olat, olon)
    #utmy, utmx, utmzone = gc.LLtoUTM(p.lat, p.lon)
    #xa, ya = axy.ll2xy(p.lat, p.lon, olat, olon)

    rospy.loginfo("#########  "+p.name+"  ###########")
    rospy.loginfo("LAT COORDINATES ==>"+str(p.lat)+","+str(p.lon))
    rospy.loginfo("COORDINATES XYZ ==>"+str(xg2)+","+str(yg2))
    #rospy.loginfo("COORDINATES AXY==>"+str(xa)+","+str(ya))
    #rospy.loginfo("COORDINATES UTM==>"+str(utmx)+","+str(utmy))

    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, p.theta)
    pose = Pose()
    pose.position.x = xg2
    pose.position.y = yg2
    q = Quaternion()
    q.x = quaternion[0]
    q.y = quaternion[1]
    q.z = quaternion[2]
    q.w = quaternion[3]
    pose.orientation = q
    return pose


class gps_point:
    lat = 0.0
    lon = 0.0
    theta = 0.0
    name = "MAP"


# initializes the action client node
rospy.init_node('move_base_gps_node')

action_server_name = '/move_base'
client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
rospy.loginfo("cazzo2")
# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

points = []
p = gps_point()
p.lat = 49.8999181588
p.lon = 8.89996774369
points.append(p)
p.lat = 49.7000059083
p.lon = 8.89999871302
points.append(p)

n = -1
rate = rospy.Rate(1)

while n < len(points) - 1:
    print("===========" + str(len(points)))
    n += 1
    rospy.loginfo("GOING TO POINT N. "+str(n))
    next_point = points[n]

    # creates a goal to send to the action server
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.get_rostime()
    goal.target_pose.pose = get_xy_based_on_lat_long(next_point)

    client.send_goal(goal, feedback_cb=feedback_callback)

    # You can access the SimpleAction Variable "simple_state", that will be 1 if active, and 2 when finished.
    # Its a variable, better use a function like get_state.
    #state = client.simple_state
    # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR

    state_result = client.get_state()

    #rospy.loginfo("state_result: "+str(state_result))

    while state_result < DONE:
        rospy.loginfo(
            "Doing Stuff while waiting for the Server to give a result....")
        rate.sleep()
        state_result = client.get_state()
        rospy.loginfo("cazzo")
        #rospy.loginfo("state_result: "+str(state_result))

    rospy.loginfo("[Result] State: "+str(state_result))
    if state_result == ERROR:
        rospy.logerr("Something went wrong in the Server Side")
    if state_result == WARN:
        rospy.logwarn("There is a warning in the Server Side")

#rospy.loginfo("[Result] State: "+str(client.get_result()))
