#!/usr/bin/env python2.7
import rospy
from udp_server import UdpServer
import logging
import sys
# import Queue
from  threading import Thread, Event
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
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
        self.write_thread = Thread(None, self.write_to_rt, "rt_interface_write_thread")

        self.odom_pub = rospy.Publisher('/encoder_odometry/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.imu_pub = rospy.Publisher('/xsens/imu', Imu, queue_size=1)
        self.cmd_vel_subs = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb, queue_size=1)
        self.cmd_vel_data = Twist()
        self.sigterm_event = Event()
        self.write_event = Event()
        self.cmd_vel_event = Event()
        self.dummy_event = Event()

        self.odom_to_bl_msg = TransformStamped()
        self.odom_to_bl_msg.header.frame_id = "odom"
        self.odom_to_bl_msg.child_frame_id = "base_footprint"

        self.odom_data = Odometry()
        self.imu_data = Imu()
        self.sigterm_event.clear()
        self.cmd_vel_event.clear()
        self.write_event.clear()
        self.dummy_event.clear()

        self.packet_state = 'header'
        self.logger.info("RtInterface Constructed")
        self.raw_data = ''
        self.packet_index = 0
        self.payload_length = 0
        # self.write_thread.start(self)

    def parse_rt_msg(self, msg):
        self.logger.debug("Raw Msg: {}".format(msg))
        header_plus_length_size = 2 + 4
        payload_size = 20
        odom_packet_size = 26
        imu_packet_size = 54
        complete_packet_size = header_plus_length_size + payload_size
        msg_bytearray = msg
        msg_len = len(msg)
        got_complete_packet = False

        if msg_len == 0:
            self.logger.info("No data in the msg")
        self.logger.info('chunk size: {}'.format(msg_len))
        self.logger.info('packet_state: {}'.format(self.packet_state))
        if self.packet_state == 'header':

            if msg_bytearray[self.packet_index] == '\xfd' and msg_bytearray[self.packet_index] == '\xfd':
                self.packet_state = 'payload_size'
                self.packet_index += 2
            else:
                self.logger.error("header: {}, junk request".format(ord(msg_bytearray[0])))

        elif self.packet_state == 'payload_size':
            if msg_len >= header_plus_length_size:
                self.payload_length = struct.unpack('<I', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                self.packet_index += 4
                self.logger.info('Payload length: {}'.format(self.payload_length))
                self.packet_state = 'payload'

        elif self.packet_state == 'payload':

            if msg_len >= self.payload_length:
                self.odom_data.header.stamp = rospy.Time.now()

                packet_id = struct.unpack('<I', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                self.packet_index += 4
                self.logger.info("Packet ID: {}".format(packet_id))
                if packet_id == 2:
                    if msg_len >= odom_packet_size:
                        timestamp = struct.unpack('<I', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        pose_x = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0] / 100.0
                        self.packet_index += 4
                        pose_y = struct.unpack('<f', msg_bytearray[self.packet_index :self.packet_index + 4])[0] / 100.0
                        self.packet_index += 4
                        pose_theta = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        twist_linear = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0] / 100.0
                        self.packet_index += 4
                        twist_angular = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4

                        self.odom_data.header.stamp = rospy.Time.now()
                        self.odom_data.header.frame_id = "odom"
                        self.odom_data.child_frame_id = "base_footprint"
                        self.odom_data.pose.pose.position.x = pose_x
                        self.odom_data.pose.pose.position.y = pose_y
                        self.odom_data.pose.pose.position.z = 0.0

                        odom_quat_trans = quaternion_from_euler(0.0, 0.0, math.radians(pose_theta))

                        self.odom_data.pose.pose.orientation.x = odom_quat_trans[0]
                        self.odom_data.pose.pose.orientation.y = odom_quat_trans[1]
                        self.odom_data.pose.pose.orientation.z = odom_quat_trans[2]
                        self.odom_data.pose.pose.orientation.w = odom_quat_trans[3]

                        self.odom_data.twist.twist.linear.x = twist_linear
                        self.odom_data.twist.twist.linear.y = 0.0
                        self.odom_data.twist.twist.linear.z = 0.0

                        self.odom_data.twist.twist.angular.x = 0.0
                        self.odom_data.twist.twist.angular.y = 0.0
                        self.odom_data.twist.twist.angular.z = twist_angular

                        got_complete_packet = True
                        self.packet_index = 0
                        self.packet_state ='header'
                        self.logger.info("Got complete packet")
                        self.logger.info("Got Odometry Data")
                        self.logger.info("position: [x: {}, y: {}, theta: {}]".format(pose_x, pose_y, pose_theta) )
                        self.logger.info("twist: [lin: {}, ang: {}]".format(twist_linear, twist_angular))
                        self.publish_odom_data()
                        self.publish_tf()

                elif packet_id == 3:
                    if msg_len >= imu_packet_size:
                        timestamp = struct.unpack('<I', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        roll = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        pitch = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        yaw = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        ang_vel_x = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        ang_vel_y = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        ang_vel_z = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        lin_acc_x = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        lin_acc_y = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4
                        lin_acc_z = struct.unpack('<f', msg_bytearray[self.packet_index:self.packet_index + 4])[0]
                        self.packet_index += 4

                        odom_quat_trans = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
                        self.imu_data.header.stamp = rospy.Time.now()
                        self.imu_data.header.frame_id = "imu_link"
                        self.imu_data.orientation.x = odom_quat_trans[0]
                        self.imu_data.orientation.y = odom_quat_trans[1]
                        self.imu_data.orientation.z = odom_quat_trans[2]
                        self.imu_data.orientation.w = odom_quat_trans[3]
                        self.imu_data.angular_velocity.x = ang_vel_x
                        self.imu_data.angular_velocity.y = ang_vel_y
                        self.imu_data.angular_velocity.z = ang_vel_z
                        self.imu_data.linear_acceleration.x = lin_acc_x
                        self.imu_data.linear_acceleration.y = lin_acc_y
                        self.imu_data.linear_acceleration.z = lin_acc_z

                        got_complete_packet = True
                        self.packet_index = 0
                        self.packet_state = 'header'
                        self.logger.info("Got complete packet")
                        self.logger.info("Got IMU Data")
                        self.logger.info("orientation: [roll: {}, pitch: {}, yaw: {}]".format(roll, pitch, yaw))
                        self.logger.info("angular_velocity : {}".format(self.imu_data.angular_velocity))
                        self.logger.info("linear_acceleration : {}".format(self.imu_data.linear_acceleration))
                        self.publish_imu_data()
                        self.logger.info("Published IMU Data")

                else:
                    got_complete_packet = False
                    self.packet_index = 0
                    self.logger.info("Packet of Unknown packet ID received: {}. Looking for Header Now".format(packet_id))
                    self.packet_state = 'header'


                self.logger.info("Residual Msg: {}".format(msg_bytearray))
                msg = ''
                self.connection_handle.raw_data = ''

            else:
                got_complete_packet = False
                self.logger.info("Incomplete packet received. Continuing with previous parser state: {}".format(self.packet_state))

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
                # if status:
                    # self.publish_odom_data()
                    # self.publish_tf()
            except KeyboardInterrupt as key_interrupt:
                self.sigterm_event.set()
                self.logger.debug("keyboard interrupt. Exiting process thread")
            except Exception as ex:
                self.logger.info("Exception: {}. Exiting process thread".format(ex))
                self.connection_handle.raw_data = ''
                self.packet_state = 'header'
                self.packet_index = 0
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
        self.odom_to_bl_msg.transform.translation.x = self.odom_data.pose.pose.position.x
        self.odom_to_bl_msg.transform.translation.y = self.odom_data.pose.pose.position.y
        self.odom_to_bl_msg.transform.translation.z = self.odom_data.pose.pose.position.z

        odom_quat = Quaternion(self.odom_data.pose.pose.orientation.x, self.odom_data.pose.pose.orientation.y,
                               self.odom_data.pose.pose.orientation.z, self.odom_data.pose.pose.orientation.w)
        odom_euler = euler_from_quaternion([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])
        roll = float(0.0)
        pitch = float(0.0)
        odom_quat_trans = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(odom_euler[2]))

        self.odom_to_bl_msg.transform.rotation.x = odom_quat_trans[0]
        self.odom_to_bl_msg.transform.rotation.y = odom_quat_trans[1]
        self.odom_to_bl_msg.transform.rotation.z = odom_quat_trans[2]
        self.odom_to_bl_msg.transform.rotation.w = odom_quat_trans[3]
        self.odom_to_bl_msg.header.stamp = rospy.Time.now()
        self.tf_broadcaster.sendTransform(self.odom_to_bl_msg)

    def publish_imu_data(self):
        self.logger.info("Publishing IMU Data")
        self.imu_pub.publish(self.imu_data)

    def cmd_vel_cb(self, cmd_vel):
        self.logger.info("Received cmd_vel: linear: {} m/s angular: {} rad/s".format(cmd_vel.linear.x, cmd_vel.angular.z))
        self.cmd_vel_data = cmd_vel
        self.cmd_vel_event.set()
        self.write_event.set()

    def write_to_rt(self):
        while self.keep_alive and not self.sigterm_event.is_set():
            self.write_event.wait()
            self.logger.info("Write event wait exit \n")
            if self.cmd_vel_event.is_set():
                self.send_cmd_vel()
                self.cmd_vel_event.clear()

            elif self.dummy_event.is_set():
                self.logger.info("Dummy Event. Clearing event flag")
                self.dummy_event.clear()

            else:
                self.logger.info("No sub-event")

            self.write_event.clear()
            self.logger.info("Clearing write event")

    def send_cmd_vel(self):
        self.logger.info("Sending cmd_vel to RT")
        cmd_vel_id = 3
        msg_str = self.pack_rt_msg(cmd_vel_id)
        self.logger.info("msg_str: {}".format(msg_str))
        self.connection_handle.send_msg(msg_str, len(msg_str))

    def pack_rt_msg(self, packet_id):
        header = '\xfd\xfd'
        min_packet_size = 14
        packet_id_str = struct.pack('<I', packet_id)
        length_str = ''
        payload_str = ''
        checksum_str = ''

        if packet_id == 3:
            payload_size = 24
            packet_length = min_packet_size + payload_size
            length_str = struct.pack('<I', packet_length)
            cmd_vel = copy.deepcopy(self.cmd_vel_data)
            payload_str = struct.pack('<d', cmd_vel.linear.x)
            payload_str += struct.pack('<d', cmd_vel.linear.y)
            payload_str += struct.pack('<d', cmd_vel.linear.z)
            payload_str += struct.pack('<d', cmd_vel.angular.x)
            payload_str += struct.pack('<d', cmd_vel.angular.y)
            payload_str += struct.pack('<d', cmd_vel.angular.z)
            checksum_data = 0
            checksum_str = struct.pack('<I', checksum_data)

        packet = header + length_str + packet_id_str + payload_str + checksum_str

        return packet






def main():
    rospy.init_node("rt_interface")
    logging.basicConfig(format='[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s', level=logging.DEBUG,
                        filename='rt_interface.log')
    logging.debug('Logger created')
    rt_interface = RtInterface('', 5102)

    print("Starting read thread")
    rt_interface.read_thread.start()
    print("Starting write thread")
    rt_interface.write_thread.start()

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
