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

SEARCH_WAYPOINTS = {'1': [(3.03781, 0.07015, 0.0), (0.0, 0.0, -0.384214755907, 0.923243749691)],
                    '2': [(4.16285, 1.27578, 0.0), (0.0, 0.0, -0.359671123975, 0.933079140576)]}

PARK_SPOT_WAYPOINTS = {'1': [(5.10959,  1.47603, 0.0), (0.0, 0.0, -0.350370856062, 0.936611052264)],
                       '2': [(4.64341,  0.88347, 0.0), (0.0, 0.0, -0.314489530921, 0.949260941439)],
                       '3': [(4.14429,  0.27720, 0.0), (0.0, 0.0, -0.302952105343, 0.953005782705)],
                       '4': [(3.69111, -0.36977, 0.0), (0.0, 0.0, -0.296874921752, 0.954916373739)],
                       '5': [(3.27664, -1.03622, 0.0), (0.0, 0.0, -0.290952676844, 0.956737445613)],
                       '6': [(3.31244,  1.39406, 0.0), (0.0, 0.0, -0.275217900052, 0.961381873914)],
                       '7': [(2.90084,  0.73873, 0.0), (0.0, 0.0, -0.273119836863, 0.961980017834)],
                       '8': [(2.51752, -0.50744, 0.0), (0.0, 0.0, -0.295958131198, 0.955200913200)]}

ON_RAMP_WAYPOINT = [(3.37777, 1.49040, 0.0),(0.0, 0.0, 0.885636895101, 0.464378391008)]

class SearchARtag(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['found_AR_tag', 'end', 'nothing_found'],
                                output_keys=['SearchARtag_out_AR_tag_pose']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
        
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else: 
            ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_callback)
            for waypoint in SEARCH_WAYPOINTS:
                goal = util.goal_pose(SEARCH_WAYPOINTS[waypoint], 'map')
                self.client.send_goal(goal)
                self.client.wait_for_result()
                found, target_pose = self.search()
                if found:
                    userdata.SearchARtag_out_AR_tag_pose = target_pose
                    ar_sub.unregister()
                    return 'found_AR_tag'
            ar_sub.unregister()
            return 'nothing_found'

    def search(self):
        for i in range(4):
            if len(self.tags) != 0:
                ARtag = self.tags[0]
                return True, ARtag
            else:
                util.rotate(angle=90)
                rospy.sleep(0.5)
        return False, None

    def ar_callback(self, msg):
        self.tags = []
        for marker in msg.markers:
            #self.tags[int(marker.id)] = marker.pose.pose
            self.tags.append((int(marker.id), marker.pose.pose))
    
class ARtagPark(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['return', 'end'],
                                input_keys=['ARtagPark_in_AR_tag_pose']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else: 
            waypoint = userdata.ARtagPark_in_AR_tag_pose
            #waypoint.position[0]
            goal = util.goal_pose(waypoint, 'map')
            self.client.send_goal(goal)
            self.client.wait_for_result()
            util.signal(1, onColor=Led.GREEN)
            return 'return'

class SearchContour(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['found_contour', 'end', 'nothing_found']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
        
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else: 
            # image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.shape_cam_callback)
            # for waypoint in SEARCH_WAYPOINTS:
            #     goal = util.goal_pose(SEARCH_WAYPOINTS[waypoint], 'map')
            #     self.client.send_goal(goal)
            #     self.client.wait_for_result()
            #     found = self.search()
            #     if found:
            #         image_sub.unregister()
            #         return 'found_contour'
            # image_sub.unregister()
            return 'nothing_found'

    def search(self):
        for i in range(4):
            cd = detectshapes.ContourDetector()
            _, red_contours = cd.getContours(self.hsv)
            if len(red_contours) > 0:
                return True
            else:
                util.rotate(angle=90)
                rospy.sleep(0.5)
        return False
    
    def shape_cam_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

class ContourPark(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['return', 'end'],
                                input_keys="ContourPark_in_contour"
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
        self.contour = detectshapes.Contour.Unidentified
        self.r_min = 0
        self.r_max = 0
        self.target_dist = 0
        self.current_pose = None
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else: 
            self.contour = userdata.ContourPark_in_contour
            self.adjust()
            self.approach()
            park_spot = self.findwaypoint()
            goal = util.goal_pose(PARK_SPOT_WAYPOINTS[park_spot], 'map')
            self.client.send_goal(goal)
            self.client.wait_for_result()
            util.signal(1, onColor=Led.ORANGE)
            return 'return'

    def shape_cam_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
        self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    def laser_callback(self, msg):
        array = numpy.array(msg.ranges[self.r_min: self.r_max+1])
        array = array[~numpy.isnan(array)]
        self.target_dist = numpy.mean(array)
    
    def odom_callback(self, msg):
        self.current_pose = (msg.geometry_msgs.pose.position.x, msg.geometry_msgs.pose.position.y)

    def adjust(self):
        self.cd = detectshapes.ContourDetector()
        image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.shape_cam_callback)
        rospy.wait_for_message("camera/rgb/image_raw", Image)
        is_in_middle = False
        while not is_in_middle:
            r_min, r_max = self.cd.get_red_contour_range(self.hsv, self.contour)
            middle = (r_max + r_min)/2
            print "middle = ", middle
            if middle!= numpy.nan:
                if util.approxEqual(middle, 320, 10):
                    self.r_min, self.r_max = r_min, r_max
                    break
                else:
                    if middle > 320:
                        util.rotate(angle=3, max_error=1, anglular_scale=0.2)
                    else:
                        util.rotate(angle=-3, max_error= 1, anglular_scale=0.2)
            else:
                util.rotate(angle=3, max_error= 1, anglular_scale=0.2)
    
    def approach(self):
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.wait_for_message('/scan', LaserScan)
        twist_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
        while self.target_dist > 0.5:
            print "self.target_dist = ", self.target_dist 
            twist = Twist()
            twist.linear.x = 0.2
            twist_pub.publish(twist)
        twist_pub.publish(Twist())
        twist_pub.unregister()
    
    def findwaypoint(self):
        odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.wait_for_message('/odom', Odometry)
        tmp_dict = {}
        for item in PARK_SPOT_WAYPOINTS:
            tmp_x = PARK_SPOT_WAYPOINTS[item][0][0] - self.current_pose[0] 
            tmp_y = PARK_SPOT_WAYPOINTS[item][0][1] - self.current_pose[1] 
            dist = numpy.sqrt(tmp_x**2 + tmp_y**2)
            tmp_dict[item] = dist
        odom_sub.unregister()
        return min(tmp_dict, key=tmp_dict.get)

class NoLabelPark(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['return', 'end'],
                                input_keys=['NoLabelPark_in_unmarked_pose']
        )
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.client.wait_for_server()
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        else: 
            goal = util.goal_pose(userdata.NoLabelPark_in_unmarked_pose, 'map')
            self.client.send_goal(goal)
            self.client.wait_for_result()
            util.signal(1, onColor=Led.RED)
            return 'return'

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
            util.signal(1, onColor=Led.ORANGE)
            return 'return'

# if __name__ == "__main__":
#     g_tags = {}
#     rospy.init_node("work4_test")

#     sm = smach.StateMachine(outcomes=['end'])
#     sm.userdata.unmarked_pose = PARK_SPOT_WAYPOINTS['1']
#     sm.userdata.shape_at_loc2 = detectshapes.Contour.Circle
#     with sm:   
#         smach.StateMachine.add('SearchARtag', SearchARtag(),
#                                 transitions={'found_AR_tag':'ARtagPark',
#                                             'end':'end',
#                                             'nothing_found':'SearchContour'
#                                             },
#                                 remapping={'SearchARtag_out_AR_tag_pose':'AR_tag_pose'})
#         smach.StateMachine.add('ARtagPark', ARtagPark(),
#                                 transitions={'return':'SearchContour',
#                                             'end':'end'
#                                             },
#                                 remapping={'ARtagPark_in_AR_tag_pose':'AR_tag_pose'})
#         smach.StateMachine.add('SearchContour', SearchContour(),
#                                 transitions={'found_contour':'ContourPark',
#                                             'end':'end',
#                                             'nothing_found':'NoLabelPark',
#                                             })
#         smach.StateMachine.add('ContourPark', ContourPark(),
#                                 transitions={'return':'NoLabelPark',
#                                             'end':'end'
#                                             },
#                                 remapping={'ContourPark_in_contour':'shape_at_loc2'})
#         smach.StateMachine.add('NoLabelPark', NoLabelPark(),
#                                 transitions={'return':'ON_RAMP',
#                                             'end':'end'
#                                             },
#                                 remapping={'NoLabelPark_in_unmarked_pose':'unmarked_pose'})
#         smach.StateMachine.add('ON_RAMP', ON_RAMP(),
#                                 transitions={'returned':'end',
#                                             'end':'end'
#                                             })
    
#     outcome = sm.execute()
#     rospy.spin()

#test ARtag
if __name__ == "__main__":
    g_tags = {}
    rospy.init_node("work4_test")

    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.unmarked_pose = PARK_SPOT_WAYPOINTS['1']
    sm.userdata.shape_at_loc2 = detectshapes.Contour.Circle
    with sm:   
        smach.StateMachine.add('SearchARtag', SearchARtag(),
                                transitions={'found_AR_tag':'ARtagPark',
                                            'end':'end',
                                            'nothing_found':'end'
                                            },
                                remapping={'SearchARtag_out_AR_tag_pose':'AR_tag_pose'})
        smach.StateMachine.add('ARtagPark', ARtagPark(),
                                transitions={'return':'end',
                                            'end':'end'
                                            },
                                remapping={'ARtagPark_in_AR_tag_pose':'AR_tag_pose'})   
    outcome = sm.execute()
    rospy.spin()      

#test UNMarked
if __name__ == "__main__":
    g_tags = {}
    rospy.init_node("work4_test")

    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.unmarked_pose = PARK_SPOT_WAYPOINTS['1']
    sm.userdata.shape_at_loc2 = detectshapes.Contour.Circle
    with sm:   
        smach.StateMachine.add('NoLabelPark', NoLabelPark(),
                                transitions={'return':'end',
                                            'end':'end'
                                            },
                                remapping={'NoLabelPark_in_unmarked_pose':'unmarked_pose'})
    outcome = sm.execute()
    rospy.spin()    

#test Contour
if __name__ == "__main__":
    g_tags = {}
    rospy.init_node("work4_test")

    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.unmarked_pose = PARK_SPOT_WAYPOINTS['1']
    sm.userdata.shape_at_loc2 = detectshapes.Contour.Circle
    with sm:   
        smach.StateMachine.add('SearchContour', SearchContour(),
                                transitions={'found_contour':'ContourPark',
                                            'end':'end',
                                            'nothing_found':'end',
                                            })
        smach.StateMachine.add('ContourPark', ContourPark(),
                                transitions={'return':'end',
                                            'end':'end'
                                            },
                                remapping={'ContourPark_in_contour':'shape_at_loc2'})
    outcome = sm.execute()
    rospy.spin() 