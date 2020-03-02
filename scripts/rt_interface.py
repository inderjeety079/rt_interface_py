#!/usr/bin/env python2.7
import rospy
from udp_server import UdpServer
import logging
import sys
# import Queue
from  threading import Thread, Event
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import traceback
# from tf2_ros import TransformBroadcaster
# import tf2_ros
import tf2_ros
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import copy
import math
from geometry_msgs.msg import Quaternion
# from geometry_msgs.msg import Pose
import struct
import time


class RtInterface:
    def __init__(self, hostname, port):
        self.logger = logging.getLogger('RtInterface')
        self.hostname = hostname
        self.port = port
        self.keep_alive = True
        self.connection_handle = UdpServer(self.hostname, self.port, 1)
        print("Assigning read thread handler")
        self.read_thread = Thread(None, self.process_rt_msg, "rt_interface_read_thread")
        # self.publish_thread = Thread(self.send_msg_to_rt)

        self.odom_pub = rospy.Publisher('/encoder_odometry/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.sigterm_event = Event()

        self.odom_to_bl_msg = TransformStamped()
        self.odom_to_bl_msg.header.frame_id = "odom"
        self.odom_to_bl_msg.child_frame_id = "base_link"

        self.odom_data = Odometry()
        self.sigterm_event.clear()

        self.packet_state = 'header'
        self.logger.info("RtInterface Constructed")
        self.raw_data = ''
        # self.write_thread.start(self)

    def parse_rt_msg(self, msg):
        self.logger.debug("Raw Msg: {}".format(msg))
        header_plus_length_size = 1 + 4
        payload_size = 20
        complete_packet_size = header_plus_length_size + payload_size
        msg_len = len(msg)
        got_complete_packet = False
        if msg_len == 0:
            self.logger.info("No data in the msg")
        self.logger.info('chunk size: {}'.format(msg_len))
        if self.packet_state == 'header':
            if msg[0] == '\xfd' and msg[1] == '\xfd':
                self.packet_state = 'payload_size'
            else:
                self.logger.error("header: {}, junk request".format(ord(msg[0])))

        if self.packet_state == 'payload_size':
            if msg_len >= header_plus_length_size:
                payload_length = struct.unpack('I', msg[2:6])[0]
                self.logger.info('Payload length: {}'.format(payload_length))
                self.packet_state = 'payload'

        if self.packet_state == 'payload':

            if msg_len >= complete_packet_size :
                self.odom_data.header.stamp = rospy.Time.now()

                pose_x = struct.unpack('f', msg[6:10])[0] / 100.0
                pose_y = struct.unpack('f', msg[10:14])[0] / 100.0
                pose_theta = struct.unpack('f', msg[14:18])[0]
                twist_linear = struct.unpack('f', msg[18:22])[0] / 100.0
                twist_angular = struct.unpack('f', msg[22:26])[0]

                self.odom_data.pose.pose.position.x = pose_x
                self.odom_data.pose.pose.position.y = pose_y
                self.odom_data.pose.pose.position.z = 0.0

                self.odom_data.pose.pose.orientation.x = 0.0
                self.odom_data.pose.pose.orientation.y = 0.0
                self.odom_data.pose.pose.orientation.z = 0.0

                self.odom_data.twist.twist.linear.x = twist_linear
                self.odom_data.twist.twist.linear.y = 0.0
                self.odom_data.twist.twist.linear.z = 0.0

                self.odom_data.twist.twist.angular.x = 0.0
                self.odom_data.twist.twist.angular.y = 0.0
                self.odom_data.twist.twist.angular.z = twist_angular

                got_complete_packet = True
                self.logger.info("Got complete packet")
                self.logger.info("position: [x: {}, y: {}, theta: {}]".format(pose_x, pose_y, pose_theta) )
                self.logger.info("twist: [lin: {}, ang: {}]".format(twist_linear, twist_angular))
                self.packet_state == 'header'
                msg = ''
                self.connection_handle.raw_data = ''


        return got_complete_packet

    def process_rt_msg(self):
        self.logger.debug("waiting for connection")
        #client_sock = self.connection_handle.accept_connection()
        self.connection_handle.bind()
        status = False

        while self.keep_alive and not self.sigterm_event.is_set():
            try:
                self.logger.info("Waiting for the data to arrive in rt interface")
                self.connection_handle.recv_msg()
                self.logger.info("Recvd msg:{}".format(self.connection_handle.raw_data))
                status = self.parse_rt_msg(self.connection_handle.raw_data)
                if status:
                    self.publish_odom_data()
                    self.publish_tf()
            except KeyboardInterrupt as key_interrupt:
                self.sigterm_event.set()
                self.logger.debug("keyboard interrupt. Exiting process thread")
            except Exception as ex:
                self.logger.info("Exception: {}. Exiting process thread".format(ex))
                time.sleep(0.1)
                # sys.exit(-1)

    # def send_msg_to_rt(self, msg):

    def engage_emergency(self):
        engage_emg_msg = '\xff\xff\x05\x01\x01\x00'
        # self.send_msg_to_rt(engage_emg_msg)

    def disengage_emergency(self):
        disengage_emg_msg = '\xff\xff\x05\x01\x02\x00'
        # self.send_msg_to_rt(disengage_emg_msg)

    # def receive_from_rt(self):

    def publish_odom_data(self):

        self.odom_pub.publish(self.odom_data)

    def publish_tf(self):
        self.odom_to_bl_msg.transform.translation.x = self.odom_data.pose.position.x
        self.odom_to_bl_msg.transform.translation.y = self.odom_data.pose.position.y
        self.odom_to_bl_msg.transform.translation.z = self.odom_data.pose.position.z

        odom_quat = Quaternion(self.odom_data.pose.pose.orientation.x, self.odom_data.pose.pose.orientation.y,
                               self.odom_data.pose.pose.orientation.z, self.odom_data.pose.pose.orientation.w)
        odom_euler = euler_from_quaternion([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])
        odom_quat_trans = quaternion_from_euler(0, 0, math.radians(odom_euler))

        self.odom_to_bl_msg.transform.rotation.x = odom_quat_trans[0]
        self.odom_to_bl_msg.transform.rotation.y = odom_quat_trans[1]
        self.odom_to_bl_msg.transform.rotation.z = odom_quat_trans[2]
        self.odom_to_bl_msg.transform.rotation.w = odom_quat_trans[3]
        self.odom_to_bl_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(self.odom_to_bl_msg)


def main():
    rospy.init_node("rt_interface")
    logging.basicConfig(format='[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s', level=logging.DEBUG,
                        filename='rt_interface.log')
    logging.debug('Logger created')
    rt_interface = RtInterface('', 5102)

    print("Starting read thread")
    rt_interface.read_thread.start()

    print("Created Rt Interface")
    print("Starting to spin")
    rospy.spin()
    rt_interface.read_thread.join()

    logging.debug('Exiting Rt Interface Node')
    print('Exiting Rt Interface Node')


if __name__ == '__main__':
    try:
        main()
        # except rospy.ROSException as ros_err:
        #     pass
    except Exception as err:
        traceback.print_exc()
    except KeyboardInterrupt as keyboard_interrupt:
        print("Keyboard Interrupt")
