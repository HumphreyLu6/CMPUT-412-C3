#!/usr/bin/env python
import smach, smach_ros, rospy, numpy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
import util, detectshapes
from kobuki_msgs.msg import Led, Sound
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import cv2, cv_bridge

SEARCH_WAYPOINTS = {'1': [(-0.822, -0.009, 0.010), (0, 0, 0.118, 0.993)],
                    '2': [(-1.313,  1.434, 0.010), (0, 0, 0.127, 0.992)]}

PARK_SPOT_WAYPOINTS = {'1': [(-0.803,  2.353, 0.010), (0, 0,  0.105,  0.994)],
                       '2': [(-0.608,  1.657, 0.010), (0, 0,  0.134,  0.991)],
                       '3': [(-0.401,  0.869, 0.010), (0, 0,  0.124,  0.992)],
                       '4': [(-0.291,  0.079, 0.010), (0, 0,  0.106,  0.994)],
                       '5': [(-0.169, -0.719, 0.010), (0, 0,  0.092,  0.996)],
                       '6': [(-1.922,  1.029, 0.010), (0, 0,  0.997, -0.080)],
                       '7': [(-1.694,  0.291, 0.010), (0, 0,  0.992, -0.123)],
                       '8': [(-1.077, -0.846, 0.010), (0, 0, -0.621,  0.784)]}

OFF_RAMP_WAYPOINT = [(-1.895, -0.507, 0.010), (0.000, 0.000, 0.156, 0.988)] #start

ON_RAMP_WAYPOINT = [(-2.901, 1.809, 0.010), (0.000, 0.000, 0.960, -0.281)] #end
    
class Park(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['next', 'end', 'return'],
                                input_keys=['Park_in_process'],
                                output_keys=['Park_in_process']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else: 
            process = userdata.Park_in_process
            waypoint = PARK_SPOT_WAYPOINTS[process['park_id']]
            goal = util.goal_pose(waypoint, 'map', 'list')
            self.client.send_goal(goal)
            self.client.wait_for_result()
            self.search(process)
            process['spot_id'] += 1
            if process['spot_id'] > 8:
                return 'return'
            else:
                return 'next'
    
    def search(self, process):
        if process['ARtag_found'] == False:
            ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_callback)
            rospy.wait_for_message("ar_pose_marker", AlvarMarkers)
            found = self.search_ARtag()
            ar_sub.unregister()
            if found:
                util.signal(1, onColor=Led.GREEN)
                process['ARtag_found'] = True
        if process['contour_found'] == False:
            image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.shape_cam_callback)
            rospy.wait_for_message("camera/rgb/image_raw", Image)
            found = self.search_contour()
            #image_sub.unregister()
            if found:
                util.signal(1, onColor=Led.ORANGE)
                process['contour_found'] = True
        if process['spot_id'] == process['unmarked_spot_id']:
            util.signal(1, onColor=Led.RED)
    
    def search_ARtag(self):
        for _ in range(4):
            if len(self.tags) != 0:
                print "tag_id:", self.tags[0]
                return True
            else:
                util.rotate(angle=-90)
                rospy.sleep(0.5)
        return False
    
    def search_contour(self):
        for i in range(4):
            cd = detectshapes.ContourDetector()
            _, red_contours = cd.getContours(self.hsv)
            if len(red_contours) > 0:
                return True
            else:
                util.rotate(angle=90)
                rospy.sleep(0.5)
        return False

    def ar_callback(self, msg):
        self.tags = []
        for marker in msg.markers:
            self.tags.append(int(marker.id))
    
    def shape_cam_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

class ON_RAMP(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['returned', 'end']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else: 
            goal = util.goal_pose(ON_RAMP_WAYPOINT)
            self.client.send_goal(goal)
            self.client.wait_for_result()
            return 'returned'

if __name__ == "__main__":
    rospy.init_node("work4_test")

    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.process = {'spot_id': 1,
                            'ARtag_found': False,
                            'contour_found': False,
                            'unmarked_spot_id':4}

    with sm:   
        smach.StateMachine.add('Park', Park(),
                                transitions={'next':'Park',
                                            'end':'ON_RAMP'
                                            },
                                remapping={'Park_in_process':'process',
                                           'Park_out_process':'process'})
        smach.StateMachine.add('ON_RAMP', ON_RAMP(),
                                transitions={'end':'end'})
    
    outcome = sm.execute()
    rospy.spin()
