import rospy
import time
import spidev
import struct
import socket
import json
import os

from rospy_message_converter import message_converter

from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32, UInt8

BAUD_RATE = 1000000 # Baud rate for SPI

"""
Communicates with the MCU to send and receive data via SPI.
"""

class MCU_Comms:
    """
    Sets up communication between the Jetson and the MCU
    """
    def __init__(self):

        # Initialize the node
        rospy.init_node('mcu_comms', anonymous=True)

        # Publish the odometry data
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

        # Publish the TF data
        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
        
        # # Publish the imu data
        # self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=10)

        self.roll_pitch_pub = rospy.Publisher("/imu/roll_pitch", Quaternion, queue_size=10)
        
        # Publish the reflective sensor data
        self.left_sensor_pub = rospy.Publisher('/cliff_sensor/left_sensor', Float32, queue_size=10)
        self.front_sensor_pub = rospy.Publisher('/cliff_sensor/front_sensor', Float32, queue_size=10)
        self.right_sensor_pub = rospy.Publisher('/cliff_sensor/right_sensor', Float32, queue_size=10)

        # Create the SPI object to facilitate SPI communication via Jetson and MCU
        self.spi = spidev.SpiDev()  # Create SPI object

        # Get ROBOT_ID from environment variable
        robot_id_env = os.getenv('ROBOT_ID')
        if robot_id_env == "1":
            self.spi.open(2,0)  # open spi port 2, device (CS) 0
        elif robot_id_env == "2":
            self.spi.open(0,0)  # open spi port 0, device (CS) 0
        else:
            self.spi.open(0,0)  # open spi port 0, device (CS) 0

            
        self.spi.max_speed_hz = BAUD_RATE  
        self.spi.mode = 0b11  # CPOL = 1, CPHA = 1 (i.e. clock is high when idle, data is clocked in on rising edge)

        self.robot_id = 0x00

        self.button_status = [False, False, False]
        # Publisher for button status
        self.button_pub = rospy.Publisher('/button_status', UInt8, queue_size=10)

        # Initialize linear/angular velocity commands
        self.lin_cmd = 0.0
        self.ang_cmd = 0.0
        
        # This is the transform to give relation between camera and robot
        self.transform_stamped_cam = TransformStamped()
                
        self.transform_stamped_cam.header.stamp = rospy.Time.now()
        self.transform_stamped_cam.header.frame_id = "base_footprint"
        self.transform_stamped_cam.header.seq = 0
        self.transform_stamped_cam.child_frame_id = "camera_link"
        self.transform_stamped_cam.transform.translation.x = 0.127
        self.transform_stamped_cam.transform.translation.y = 0
        self.transform_stamped_cam.transform.translation.z = 0
                
        rotation = quaternion_from_euler(0,0,0)
        rotation = Quaternion(*rotation)        
        self.transform_stamped_cam.transform.rotation.x = rotation.x
        self.transform_stamped_cam.transform.rotation.y = rotation.y
        self.transform_stamped_cam.transform.rotation.z = rotation.z
        self.transform_stamped_cam.transform.rotation.w = rotation.w        

        # Subscribe to the cmd_vel topic to receive velocity commands
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)


    def mcu_startup(self, pos_x=None, pos_y=None, pos_theta=None):
        """
        This function is used to bring up the MCU online and confirm communication
        """

        # Bringup message to MCU
        bringup_message = [90, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Send bringup message to MCU until received and confirmed
        bringup_confirmed = False
        while not bringup_confirmed:
            bringup_message = [90, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.spi.writebytes([55])
            rcvd = self.spi.xfer(bringup_message)
            print(rcvd)

            # Check if the MCU has confirmed bringup
            if rcvd[0] == 0 and rcvd[1] == 255 and rcvd[2] == 0:
                bringup_confirmed = True  # MattBot is active
                self.robot_id = rcvd[3]  # Get and store the robot ID
                print("Robot ID: " + str(self.robot_id))
            time.sleep(0.1)

        # Send confirmation message to MCU
        if pos_x is None and pos_y is None and pos_theta is None:
            confirmation_message = [90, 170, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        else:
            # Convert position data to bytes
            pos_x_bytes = float_to_bytes(pos_x)
            pos_y_bytes = float_to_bytes(pos_y)
            pos_theta_bytes = float_to_bytes(pos_theta)

            confirmation_message = [90, 170, pos_x_bytes[3], pos_x_bytes[2], pos_x_bytes[1], pos_x_bytes[0],
                                    pos_y_bytes[3], pos_y_bytes[2], pos_y_bytes[1], pos_y_bytes[0],
                                    pos_theta_bytes[3], pos_theta_bytes[2], pos_theta_bytes[1], pos_theta_bytes[0],
                                    0, 0]

        # Send the confirmation message to the MCU
        self.spi.writebytes([55])
        rcvd = self.spi.xfer2(confirmation_message)
        print(rcvd)

    def vel_callback(self, data):
        """
        This function is called whenever a new cmd_vel message is received
        """
        self.lin_cmd = data.linear.x
        self.ang_cmd = data.angular.z

    def send_vel_command(self):
        """
        This is just used for testing communication with the MCU
        """
        n = 0
        while n < 20:
            vel_msg = [45,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,10,11,12,13,14,15,16]
            rcvd = self.spi.xfer(vel_msg)
            print(rcvd)
            n = n + 1
            time.sleep(0.01)
        time.sleep(2.5)
        end_msg = [90, 0b11110000,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        rcvd = self.spi.xfer(end_msg)
        print(rcvd)

    def resync(self):
        """
        Used to resync when we misalign with MCU
        """
        for ii in range(20):
            self.spi.writebytes(66)
        self.spi.writebytes(77)
        self.spi.writebytes(55)
       

    def run(self):
        """
        This is the main loop for the MCU communication node
        """
        self.mcu_startup()
        
        # Execute loop at 300 Hz: every topic published at 300/6=50 Hz
        rate = rospy.Rate(200)

        # Variables to save throughout the loop
        acc_x = 0
        acc_y = 0
        ang_vel_z = 0
        qx = 0
        qy = 0

        num_unknown = 0

        sensor_sequence = 0  # Sequence number for sensor messages
        while not rospy.is_shutdown():
            # Send velocity command to MCU
            lin_vel_bytes = float_to_bytes(self.lin_cmd)
            ang_vel_bytes = float_to_bytes(self.ang_cmd)
            vel_msg = [45,  # Indicates velocity message
                       lin_vel_bytes[3],lin_vel_bytes[2],lin_vel_bytes[1],lin_vel_bytes[0],  # Linear velocity
                       ang_vel_bytes[3],ang_vel_bytes[2],ang_vel_bytes[1],ang_vel_bytes[0],  # Angular velocity
                       0,0,0,0,0,0,0]  # Padding
            self.spi.writebytes([55])
            time.sleep(0.000001)
            rcvd = self.spi.xfer(vel_msg)
            # print(rcvd)

            # Now do something with the received data
            if rcvd[0] == 7: # Received dead reckoning data
                num_unknown = 0  # Reset unknown message count

                # Extract the dead reckoning data (converts from bytes to float)
                V_dr = bytes_to_float(list(reversed(rcvd[1:5])))
                w_dr = bytes_to_float(list(reversed(rcvd[5:9])))

                # Load the data into an Odometry Message
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                odom.header.frame_id = "odom"
                odom.header.seq = sensor_sequence
                odom.child_frame_id = "base_footprint"
                
                odom.pose.pose.position.x = pos_x
                odom.pose.pose.position.y = pos_y
                odom.pose.pose.position.z = 0
                
                rotation = quaternion_from_euler(0,0, pos_theta)
                rotation = Quaternion(*rotation)
                
                odom.pose.pose.orientation.x = rotation.x
                odom.pose.pose.orientation.y = rotation.y
                odom.pose.pose.orientation.z = rotation.z
                odom.pose.pose.orientation.w = rotation.w
                
                odom.twist.twist.linear.x = V_dr
                odom.twist.twist.linear.y = 0
                odom.twist.twist.linear.z = 0
                
                odom.twist.twist.angular.x = 0
                odom.twist.twist.angular.y = 0
                odom.twist.twist.angular.z = w_dr

                self.odom_pub.publish(odom)  # actually publish the data

                sensor_sequence = sensor_sequence + 1

            elif rcvd[0] == 8:  # Recieved position data
                num_unknown = 0  # Reset unknown message count

                # Extract the position data (converts from bytes to float)
                pos_x = bytes_to_float(list(reversed(rcvd[1:5])))
                pos_y = bytes_to_float(list(reversed(rcvd[5:9])))
                pos_theta = bytes_to_float(list(reversed(rcvd[9:13])))

                # Load the data into a TF Message
                tf_msg = TFMessage()
                transform_stamped = TransformStamped()
                
                transform_stamped.header.stamp = rospy.Time.now()
                transform_stamped.header.frame_id = "odom"
                transform_stamped.header.seq = sensor_sequence
                transform_stamped.child_frame_id = "base_footprint"
                transform_stamped.transform.translation.x = pos_x
                transform_stamped.transform.translation.y = pos_y
                transform_stamped.transform.translation.z = 0
                
                rotation = quaternion_from_euler(0,0, pos_theta)
                rotation = Quaternion(*rotation)
                
                transform_stamped.transform.rotation.x = rotation.x
                transform_stamped.transform.rotation.y = rotation.y
                transform_stamped.transform.rotation.z = rotation.z
                transform_stamped.transform.rotation.w = rotation.w

                tf_msg.transforms.append(transform_stamped)  # Add the new TF
                
                self.transform_stamped_cam.header.stamp = rospy.Time.now()
                tf_msg.transforms.append(self.transform_stamped_cam)

                self.tf_pub.publish(tf_msg)  # actually publish the data

            elif rcvd[0] == 9: # Received IMU data
                num_unknown = 0  # Reset unknown message count

                roll = bytes_to_float(list(reversed(rcvd[1:5])))
                pitch = bytes_to_float(list(reversed(rcvd[5:9])))
                
                roll_pitch = quaternion_from_euler(roll, pitch, 0)
                roll_pitch = Quaternion(*roll_pitch)
                self.roll_pitch_pub.publish(roll_pitch)  # actually publish the data

                # imu = Imu()
                # # provide header information
                # imu.header.stamp = rospy.Time.now()
                # imu.header.frame_id = "imu"
                # imu.header.seq = sensor_sequence
                
                # # Load the linear accel data
                # imu.linear_acceleration.x = acc_x
                # imu.linear_acceleration.y = acc_y
                # imu.linear_acceleration.z = 0
                
                # # Load the angular velocity data
                # imu.angular_velocity.x = 0
                # imu.angular_velocity.y = 0
                # imu.angular_velocity.z = ang_vel_z
                
                # # Don't have orientation estimate so set this as the flag
                # imu.orientation_covariance[0] = -1.0
                
                # self.imu_pub.publish(imu)  # actually publish the data                

            elif rcvd[0] == 15:  # Received IMU Orientation XY data
                num_unknown = 0  # Reset unknown message count

                qx = bytes_to_float(list(reversed(rcvd[1:5])))
                qy = bytes_to_float(list(reversed(rcvd[5:9])))

            elif rcvd[0] == 16:  # Received IMU Orientation ZW data
                num_unknown = 0  # Reset unknown message count

                qz = bytes_to_float(list(reversed(rcvd[1:5])))
                qw = bytes_to_float(list(reversed(rcvd[5:9])))

                imu = Imu()
                # provide header information
                imu.header.stamp = rospy.Time.now()
                imu.header.frame_id = "imu"
                imu.header.seq = sensor_sequence
                
                # Load the linear accel data
                imu.linear_acceleration.x = acc_x
                imu.linear_acceleration.y = acc_y
                imu.linear_acceleration.z = 0
                
                # Load the angular velocity data
                imu.angular_velocity.x = 0
                imu.angular_velocity.y = 0
                imu.angular_velocity.z = ang_vel_z
                
                # Load orientation quaternion
                imu.orientation.x = qx
                imu.orientation.y = qy
                imu.orientation.z = qz
                imu.orientation.w = qw

                imu.orientation_covariance[0] = 0.0001
                imu.orientation_covariance[4] = 0.0001
                imu.orientation_covariance[8] = 0.0001
                
                self.imu_pub.publish(imu)  # actually publish the data
                
            elif rcvd[0] == 10: # Received reflective sensor data
                num_unknown = 0  # Reset unknown message count

                right_sensor = bytes_to_unsigned_int(rcvd[1], rcvd[2])
                front_sensor = bytes_to_unsigned_int(rcvd[3], rcvd[4])
                left_sensor = bytes_to_unsigned_int(rcvd[5], rcvd[6])
                
                # Actually publish the data
                self.right_sensor_pub.publish(float(right_sensor))
                self.front_sensor_pub.publish(float(front_sensor))
                self.left_sensor_pub.publish(float(left_sensor))
                button_status = rcvd[7]
                # Bits of button status indicates if each button is pressed
                button1_pressed = (button_status & 0b00000001) == 0b00000001
                button2_pressed = (button_status & 0b00000010) == 0b00000010
                button3_pressed = (button_status & 0b00000100) == 0b00000100
                
                if (button1_pressed != self.button_status[0]):
                    self.button_status[0] = button1_pressed
                    self.button_pub.publish(button_status)
                    if button1_pressed:
                        print("Button 1 pressed")
                    else:
                        print("Button 1 released")
                if (button2_pressed != self.button_status[1]):
                    self.button_status[1] = button2_pressed
                    self.button_pub.publish(button_status)
                    if button2_pressed:
                        print("Button 2 pressed")
                    else:
                        print("Button 2 released")
                if (button3_pressed != self.button_status[2]):
                    self.button_status[2] = button3_pressed
                    self.button_pub.publish(button_status)
                    if button3_pressed:
                        print("Button 3 pressed")
                    else:
                        print("Button 3 released")
            else:
                num_unknown += 1

                if num_unknown >= 20:
                    print("Resetting")
                    self.mcu_startup(pos_x=pos_x, pos_y=pos_y, pos_theta=pos_theta)
                    num_unknown = 0                
            rate.sleep()

    def shutdown(self):
        """
        This function is called when the node is shutdown
        """
        # Send shutdown message to MCU
        self.spi.writebytes([55])
        shutdown_message = [90, 0b11110000,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        rcvd = self.spi.xfer(shutdown_message)

        self.spi.close()

def float_to_bytes(float_number):
    """
    This function takes a 32-bit float and returns a list of four 8-bit integers
    """
    # Pack the 32-bit float into bytes
    packed_data = struct.pack('f', float_number)

    # Unpack the bytes into four 8-bit integers
    int_list = struct.unpack('BBBB', packed_data)

    return int_list
    
def bytes_to_float(byte_array):
    # Pack the bytes into a 32-bit float
    float_number = struct.unpack('f', bytes(byte_array))[0]

    return float_number
    
def bytes_to_unsigned_int(high_byte, low_byte):
    # Pack the bytes into a 16 bit unsigned int
    return (high_byte << 8) | low_byte

def mcu_shutdown():
    print("Shutting down MCU communication")
    comms.shutdown()

if __name__ == "__main__":
    comms = MCU_Comms()
    rospy.on_shutdown(mcu_shutdown)
    comms.run()
