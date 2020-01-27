#!/usr/bin/env python

from find_path import *
from wander_bot import *
from navigation_try.srv import *
import rospy
import tf
import std_msgs

#globals
response = None
robot_size = 0.5

'''
    handle service
'''
def handle_navigate(req):
    goal_location = [req.x,req.y]
    (robot_x,robot_y,robot_z) = robot_curr_location()
    starting_location = [robot_x,robot_y]

    # creating path finder instance
    my_find_path = find_path(response.map, starting_location, goal_location, robot_size)

    # printing new grid
    my_find_path.print_new_grid_to_file()

    # getting path
    path_to_go = my_find_path.find_shortest_path()

    # smoothing path
    (smoothed,dct) = my_find_path.smooth_path(path_to_go)

    forward_speed = 0.25
    rotation_speed = 25 * pi / 360
    min_angle = -30 * pi / 360
    max_angle = 30 * pi / 360
    my_stopper = Stopper(forward_speed, rotation_speed, min_angle, max_angle)

    for i in range(len(smoothed)-1):
        if (not dct[smoothed[i]] == None):
            dist,desired_angle = my_stopper.getDistAndAngleFromPoints(smoothed[i],dct[smoothed[i]])
            my_stopper.rotateTillDesiredAngleReached(robot_z,desired_angle)
            my_stopper.move_forward(dist)

            (robot_x, robot_y, robot_z) = robot_curr_location()

            dist, desired_angle = my_stopper.getDistAndAngleFromPoints(dct[smoothed[i]],smoothed[i+1])
            my_stopper.rotateTillDesiredAngleReached(robot_z,desired_angle)
            my_stopper.move_forward(dist)

            (robot_x, robot_y, robot_z) = robot_curr_location()

    return navigationsrvResponse(True)

'''
    main service
'''
def navigate_server():
    while (True):
        s = rospy.Service('navigate', navigationsrv, handle_navigate)
        rospy.loginfo("###########################################################")
        rospy.loginfo("ready for next goal")
        rospy.loginfo("###########################################################")
        rospy.spin()


'''
    finding robot location
'''
def robot_curr_location():
    # creating a position listener
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))

    robot_x = 0
    robot_y = 0
    robot_z = 0
    try:
        (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
        euler = tf.transformations.euler_from_quaternion(rot)
        robot_z = euler[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
        rospy.logerr("Service call failed: %s" % e)

    return [robot_x,robot_y,robot_z]

'''
    main func
'''
if __name__ == "__main__":
    rospy.init_node('navigation_task', argv=sys.argv)

    if rospy.has_param('~robot_size'):
        robot_size = rospy.get_param('~robot_size')


    # getting static map
    rospy.wait_for_service('static_map')
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)

    navigate_server()







