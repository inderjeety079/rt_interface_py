#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist
import logging
import time
import copy
import traceback



class CmdVelPublisher:
    def __init__(self):
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0

        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.prev_cmd_vel = Twist()

    def publish_cmd(self):
        self.cmd_vel.linear.x = self.prev_cmd_vel.linear.x + 0.1

        if self.cmd_vel.linear.x > 1.0:
            self.cmd_vel.linear.x = self.cmd_vel.linear.x - 0.5
        if self.cmd_vel.linear.x < 0.0:
            self.cmd_vel.linear.x = self.cmd_vel.linear.x + 0.5

        self.cmd_vel_pub.publish(self.cmd_vel)
        self.prev_cmd_vel = copy.deepcopy(self.cmd_vel)
        print ("cmd_vel:")
        print(self.cmd_vel.linear.x)




def main():
    logging.basicConfig(format='[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s', level=logging.DEBUG,
                        filename='cmd_vel_publisher.log')
    rospy.init_node("cmd_vel_publisher")
    logging.debug('Logger created')
    prev_cmd_speed = 0.0
    cmd_vel_publisher = CmdVelPublisher()

    while not rospy.is_shutdown():
        cmd_vel_publisher.publish_cmd()
        time.sleep(0.1)

    print('Exiting Rt Interface Node')

if __name__ == '__main__':
    try:
        main()
    except Exception as err:
        traceback.print_exc()


