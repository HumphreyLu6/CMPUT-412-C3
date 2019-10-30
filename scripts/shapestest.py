import ros, rospy, numpy, cv2, cv_bridge
from sensor_msgs.msg import Image
from detectshapes import ContourDetector

def threshold_hsv_360(s_min, v_min, h_max, s_max, v_max, h_min, hsv):
    lower_color_range_0 = numpy.array([0, s_min, v_min],dtype=float)
    upper_color_range_0 = numpy.array([h_max/2., s_max, v_max],dtype=float)
    lower_color_range_360 = numpy.array([h_min/2., s_min, v_min],dtype=float)
    upper_color_range_360 = numpy.array([360/2., s_max, v_max],dtype=float)
    mask0 = cv2.inRange(hsv, lower_color_range_0, upper_color_range_0)
    mask360 = cv2.inRange(hsv, lower_color_range_360, upper_color_range_360)
    mask = mask0 | mask360
    return mask

def image_callback(msg):
    global g_hsv
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
    #image = cv2.pyrMeanShiftFiltering(image, 15, 20)  #10 20 for red
    g_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #green mask
    #lower_green = numpy.array([40, 50, 50]) 
    #upper_green = numpy.array([80, 255, 255])
    #green_mask = cv2.inRange(g_hsv, lower_green, upper_green)

    #red mask
    #red_mask = threshold_hsv_360(150, 100, 20, 255, 255, 320, g_hsv) #20, 320

    #cv2.imshow("g_hsv", g_hsv)

    #cv2.imshow("green_mask", green_mask)    
    #cv2.moveWindow("green_mask", 710, 0)

    #cv2.imshow("red_mask", red_mask)
    #cv2.moveWindow("red_mask", 710, 0) 
    #cv2.waitKey(3)

import time, sys

from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

def publish_sound_led(quantity):
    global sound_pub, led_pub_1, led_pub_2
    if quantity == 1:
        led = Led()
        led.value = 0  #0 light off black
        led_pub_1.publish(Led.GREEN)
        print "q = 1"
    elif quantity == 2:
        led_pub_2.publish(Led.GREEN)
    else:
        led_pub_1.publish(Led.GREEN)
        led_pub_2.publish(Led.GREEN)
        
    for i in range(quantity):
        sound_pub.publish() # 1
        rospy.sleep(1)

def main(argv):
    global cd, g_hsv
    cd = ContourDetector()
    rospy.init_node("test_node")
    rospy.Subscriber("camera/rgb/image_raw", Image, image_callback)
    

    s_time = time.time()
    while not rospy.is_shutdown():
        if time.time() - s_time >= 4:
            cd.getContours(g_hsv, argv[0], int(argv[1]))
            s_time = time.time()
    rospy.spin()

    # global sound_pub, led_pub_1, led_pub_2
    # rospy.init_node("test_node")
    # led_pub_1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
    # led_pub_2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
    # sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)
    # s = time.time()
    # while time.time() - s <= 5:
    #     publish_sound_led(int(argv[0]))
    #     rospy.sleep(2)
    # rospy.spin()

if __name__ == "__main__":
    cd = ContourDetector()
    g_hsv = None

    sound_pub = None
    led_pub_1 = None
    led_pub_2 = None
    main(sys.argv[1:])