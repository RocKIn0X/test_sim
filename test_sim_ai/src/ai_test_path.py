#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool

class PathSim (object):

    def __init__ (self):
        print "Mission : Path"

        self.angle = 0
        self.stopt = True

        rospy.init_node ('test_sim')
        self.command = rospy.Publisher ('/cmd_vel', Twist, queue_size = 10)
        self.turn_yaw_rel = rospy.Publisher('/fix/rel/yaw', Float64, queue_size = 10)

        #path_srv = 'vision_node'
        #rospy.wait_for_service (path_srv)
        #self.detect_path = rospy.ServiceProxy (path_srv, vision_srv)

    def listToTwist (self, list):
        temp = Twist()
        temp.linear.x = list[0]
        temp.linear.y = list[1]
        temp.linear.z = list[2]
        temp.angular.x = list[3]
        temp.angular.y = list[4]
        temp.angular.z = list[5]
        return temp

    def stop_turn (self):
        return self.stopt

    def turn_yaw_relative (self, degree):
        rad = math.radians (degree)
        rad = Float64(rad)
        self.turn_yaw_rel.publish(rad)
        while not self.stop_turn():
            rospy.sleep(0.1)
        print 'turn_yaw_relative'

    def pub (self, tw):
        for i in xrange(5):
            self.command.publish (tw)
            rospy.sleep (0.05)

    def drive (self, list):
        self.pub (self.listToTwist(list))

    def stop (self, time):
        self.pub (self.listToTwist([0, 0, 0, 0, 0, 0]))
        rospy.sleep (time)

    def run (self):
        self.angle = -45
        print 'Start Sim_path Mission'

        while (True):
            self.drive ([1, 0, 0, 0, 0, 0])
            self.turn_yaw_relative(self.angle)
            self.stop (0.5)

if __name__ == '__main__':
    path_sim = PathSim()
    path_sim.run()
