#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from test_sim_srv_msg.msg import vision_data
from test_sim_srv_msg.srv import path_sim

class PathSim (object):

    def __init__ (self):
        print "Mission : Path"

        self.angle = 0
        self.data = None
        #self.stopt = True

        path_srv = 'vision'
        rospy.wait_for_service (path_srv)
        self.detect_path = rospy.ServiceProxy (path_srv, path_sim)

        rospy.init_node ('test_sim')
        self.command = rospy.Publisher ('/cmd_vel', Twist, queue_size = 10)
        self.turn_yaw_rel = rospy.Publisher('/fix/rel/yaw', Float64, queue_size = 10)

    def listToTwist (self, list):
        temp = Twist()
        temp.linear.x = list[0]
        temp.linear.y = list[1]
        temp.linear.z = list[2]
        temp.angular.x = list[3]
        temp.angular.y = list[4]
        temp.angular.z = list[5]
        return temp

    #def stop_turn (self):
    #    return self.stopt

    def turn_yaw_relative (self, degree):
        rad = math.radians (degree)
        rad = Float64(rad)
        self.turn_yaw_rel.publish(rad)
        print 'turn_yaw_relative'

        rospy.sleep(0.1)

    def pub (self, tw):
        for i in xrange(5):
            self.command.publish (tw)
            rospy.sleep (0.05)

    def drive (self, list):
        self.pub (self.listToTwist(list))

    def stop (self, time):
        self.pub (self.listToTwist([0, 0, 0, 0, 0, 0]))
        rospy.sleep (time)

    def is_center (self, data):
        if 0.3 > data.x > -0.3 and 0.3 > data.y > -0.3:
            print "center"
            return True
        return False

    def run (self):
        print 'Start Sim_path Mission'
        
        path = 'path'
        color = 'red'
        # self.drive ([1, 0, 0, 0, 0, 0])
        # rospy.sleep(11)
        # self.stop(0.1)

        while not rospy.is_shutdown():
            try:
                self.data = self.detect_path(String('path'), String('red'))
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            self.data = self.data.data
            print self.data.x
            print self.data.y
            print self.data.angle
            self.angle = self.data.angle

            self.turn_yaw_relative(self.angle)
            rospy.sleep(5)

            if self.is_center(self.data):
                self.drive ([1, 0, 0, 0, 0, 0])
                rospy.sleep(1)
            else:
                self.drive ([-self.data.x, -self.data.y, 0, 0, 0, 0])
                rospy.sleep(0.5)

            # self.drive ([5, 0, 0, 0, 0, 0])
            # self.turn_yaw_relative(self.angle)
            # self.stop (0.1)

if __name__ == '__main__':
    path_sim = PathSim()
    #path_sim.run()
    path_sim.stop(0.1)
