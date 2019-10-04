#!/usr/bin/env python

from jsk_rviz_plugins.msg import OverlayText, OverlayMenu
from std_msgs.msg import ColorRGBA, Float32, Float64
import rospy
import math
import random
from geometry_msgs.msg import Twist

class HaroOverlay(object):
    def __init__(self):
        self.plot_value_pub = rospy.Publisher("/off_path_display", Float32, queue_size=1)
        self.plot_value = 0

        self.subs = rospy.Subscriber("off_path_error", Float64, self.off_path_callback)
        
        self.counter = 0
        self.rate = rospy.Rate(10)


    def off_path_callback(self, msg):
        self.plot_value = msg.data


    def start_dummy_demo(self):
        
        while not rospy.is_shutdown():
            self.plot_value_pub.publish(self.plot_value)
            
            self.rate.sleep()
            self.counter += 1
 
    def start_harodistance_demo(self):
        
        while not rospy.is_shutdown():
            
            self.plot_value_pub.publish(self.plot_value)

            
            self.rate.sleep()
            self.counter += 1
  


  
def dummy_overlay_demo():
    rospy.init_node('distance_overlay_demo_node', anonymous=True)
    haro_overlay_object = HaroOverlay()
    try:
        #haro_overlay_object.start_dummy_demo()
        haro_overlay_object.start_harodistance_demo()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    dummy_overlay_demo()

