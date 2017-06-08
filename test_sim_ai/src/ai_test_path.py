#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, String
from test_sim_srv_msg.msg import vision_data
from test_sim_srv_msg.srv import path_sim


class PathSim (object):

    def __init__(self):
        print "Mission : Path"

        self.angle = 0
        self.data = None
        #self.stopt = True

        path_srv = 'vision'
        rospy.wait_for_service(path_srv)
        self.detect_path = rospy.ServiceProxy(path_srv, path_sim)

        rospy.init_node('test_sim')
        self.command = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.turn_yaw_rel = rospy.Publisher(
            '/fix/rel/yaw', Float64, queue_size=10)

    def listToTwist(self, list):
        temp = Twist()
        temp.linear.x = list[0]
        temp.linear.y = list[1]
        temp.linear.z = list[2]
        temp.angular.x = list[3]
        temp.angular.y = list[4]
        temp.angular.z = list[5]
        return temp

    # def stop_turn (self):
    #    return self.stopt

    def turn_yaw_relative(self, degree):
        rad = math.radians(degree)
        rad = Float64(rad)
        self.turn_yaw_rel.publish(rad)
        print 'turn_yaw_relative'

        rospy.sleep(0.1)

    def pub(self, tw):
        for i in xrange(5):
            self.command.publish(tw)
            rospy.sleep(0.05)

    def drive(self, list):
        self.pub(self.listToTwist(list))

    def stop(self, time):
        self.pub(self.listToTwist([0, 0, 0, 0, 0, 0]))
        rospy.sleep(time)

    def is_center(self, x, y):
        if 0.2 > x > -0.2 and 0.2 > y > -0.2:
            print "CENTER"
            return True
        return False

    def run(self):
        print 'Start Sim_path Mission'

        path = 'path'
        color = 'red'
        vx = 0
        vy = 0
        
        # self.drive([1, 0, 0, 0, 0, 0])
        # rospy.sleep(11)
        # self.stop(0.2)

        while not rospy.is_shutdown():
            try:
                px = 0
                py = 0
                area = 0
                angle = 0

                for i in range(5):
                    self.data = self.detect_path(String(path), String(color))
                    self.data = self.data.data
                    px = px + self.data.x
                    py = py + self.data.y
                    area = area + self.data.area
                    angle = angle + self.data.angle
                    rospy.sleep(0.2)

                px = px / 5
                py = py / 5
                area =area / 5
                angle = angle / 5

                print ('---------------')
                print ('x: ', px)
                print ('y: ', py)
                print ('area: ', area)
                print ('angle: ', angle)

                if not self.data.isFound:
                    print 'NOT FOUND PATH'

                    self.drive([0.5, 0, 0, 0, 0, 0])
                    rospy.sleep(1)

                else:
                    print 'FOUND PATH'

                    if self.is_center(px, py):
                        self.stop(0.2)
                        self.turn_yaw_relative(angle)
                        rospy.sleep(0.5)

                        print 'FOUND PATH COMPLETE'
                        break
                    else:
                        if px > 0.6:
                            vx = 0.5
                        elif px > 0.3:
                            vx = 0.3
                        elif px > 0:
                            vx = 0.1
                        else:
                            vx = -0.2

                        if py > 0.6:
                            vy = 0.5
                        elif py > 0.3:
                            vy = 0.3
                        elif py > 0:
                            vy = 0.1
                        else:
                            vy = -0.2

                        self.drive([-vx, -vy, 0, 0, 0, 0])
                    rospy.sleep(0.1)
                    
                self.stop(0.1)

                # ---- old code ----
                # self.angle = self.data.angle
                # rospy.sleep(5)

                # if self.is_center(self.data.y):
                #     if (self.data.x > -0.3):
                #         self.drive([1, 0, 0, 0, 0, 0])
                #     elif (self.data.x > -0.6):
                #         self.drive([0.5, 0, 0, 0, 0, 0])
                #     else:
                #         self.stop(0.1)
                #         print 'FIND PATH COMPLETE'
                #         break
                    
                #     rospy.sleep(1)
                # else:
                #     if (-self.data.y > 0):
                #         print 'GO LEFT'

                #         if (-self.data.y > 0.3):
                #             self.drive([0, 0.3, 0, 0, 0, 0])
                #         else:
                #             self.drive([0, -self.data.y, 0, 0, 0, 0])
                #     else:
                #         print 'GO RIGHT'

                #         if (-self.data.y < 0.3):
                #             self.drive([0, -0.3, 0, 0, 0, 0])
                #         else:
                #             self.drive([0, -self.data.y, 0, 0, 0, 0])

                #     rospy.sleep(0.5)
                
                # # self.drive([0.3, 0, 0, 0, 0, 0])
                # # self.stop(0.1)
                # self.turn_yaw_relative(self.angle)
                # rospy.sleep(0.3)

            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                break

            # self.drive ([5, 0, 0, 0, 0, 0])
            # self.turn_yaw_relative(self.angle)
            # self.stop (0.1)


if __name__ == '__main__':
    path_sim = PathSim()
    path_sim.run()
    path_sim.stop(0.1)
