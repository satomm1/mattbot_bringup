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
from std_msgs.msg import Bool, Float32, UInt8
from mattbot_bringup.msg import AirQuality

BAUD_RATE = 1000000 # Baud rate for SPI
SPI_SYNC = 55  # Sync byte prepended to every 16-byte MCU payload

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

        # Publish the air quality data
        self.air_quality_pub = rospy.Publisher("/air_quality", AirQuality, queue_size=10)

        self.roll_pitch_pub = rospy.Publisher("/imu/roll_pitch", Quaternion, queue_size=10)
        
        # Publish the reflective sensor data
        self.left_sensor_pub = rospy.Publisher('/cliff_sensor/left_sensor', Float32, queue_size=10)
        self.front_sensor_pub = rospy.Publisher('/cliff_sensor/front_sensor', Float32, queue_size=10)
        self.right_sensor_pub = rospy.Publisher('/cliff_sensor/right_sensor', Float32, queue_size=10)

        # Create the SPI object to facilitate SPI communication via Jetson and MCU
        self.spi = spidev.SpiDev()  # Create SPI object

        # Get ROBOT_ID from environment variable
        robot_id_env = os.getenv('ROBOT_ID')

        # Get MCU_SPI from environment variable
        mcu_id = os.getenv('MCU_SPI', '3')  # Default to '3' if not set

        # Open the correct SPI port based on the mcu_id, (spi3 by default)
        if mcu_id == "1":
            self.spi.open(0,0)  # open spi port 0, device (CS) 0
        elif mcu_id == "3":
            self.spi.open(2,0)  # open spi port 2, device (CS) 0
        else:
            self.spi.open(0,0)  # open spi port 0, device (CS) 0

            
        self.spi.max_speed_hz = BAUD_RATE  
        self.spi.mode = 0b11  # CPOL = 1, CPHA = 1 (i.e. clock is high when idle, data is clocked in on rising edge)

        self.robot_id = 0x00

        self.button_status = [False, False, False]
        # Publisher for button status
        self.button_pub = rospy.Publisher('/button_status', UInt8, queue_size=10)

        # Latched MCU connection state for DDS heartbeat bridging
        self.mcu_connected_pub = rospy.Publisher(
            "/mcu_connected", Bool, queue_size=1, latch=True
        )
        self._set_mcu_connected(False)

        # Initialize linear/angular velocity commands
        self.lin_cmd = 0.0
        self.ang_cmd = 0.0

        # Subscribe to the cmd_vel topic to receive velocity commands
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)

    def _set_mcu_connected(self, connected):
        """Publish latched MCU handshake / SPI-loop connection state."""
        self.mcu_connected_pub.publish(Bool(data=connected))

    def _spi_exchange(self, payload):
        """
        One CS assertion: sync byte (55) plus 16-byte payload.
        Returns 17 MISO bytes; use _mcu_msg() to get the 16-byte MCU reply.
        """
        return self.spi.xfer([SPI_SYNC] + payload + [0x00])

    @staticmethod
    def _mcu_msg(rcvd):
        """
        Skip rcvd[0] (MISO during sync clock); return the 16-byte MCU message.
        """
        # print("Received from MCU:", rcvd)
        return rcvd[2:18]

    def _recover_mcu(self, pos_x=0, pos_y=0, pos_theta=0):
        """
        Stop motion, tell MCU to shut down, wait for its watchdog, then re-handshake.
        """
        self._set_mcu_connected(False)
        self.lin_cmd = 0.0
        self.ang_cmd = 0.0
        shutdown_message = [90, 0b11110000] + [0] * 14
        for _ in range(2):
            self._spi_exchange(shutdown_message)
        time.sleep(1.1)
        self.mcu_startup(pos_x=pos_x, pos_y=pos_y, pos_theta=pos_theta)

    def mcu_startup(self, pos_x=None, pos_y=None, pos_theta=None):
        """
        This function is used to bring up the MCU online and confirm communication
        """
        self._set_mcu_connected(False)

        # Bringup message to MCU
        bringup_message = [90, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Send bringup message to MCU until received and confirmed
        bringup_confirmed = False
        while not bringup_confirmed:
            bringup_message = [90, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            rcvd = self._spi_exchange(bringup_message)
            msg = self._mcu_msg(rcvd)
            # print(msg)

            # Check if the MCU has confirmed bringup
            if msg[0] == 0 and msg[1] == 255 and msg[2] == 0:
                bringup_confirmed = True  # MattBot is active
                self.robot_id = msg[3]  # Get and store the robot ID
                rospy.loginfo("[MCU Comms] Robot ID: " + str(self.robot_id))
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
        self._spi_exchange(confirmation_message)
        self._set_mcu_connected(True)

    def vel_callback(self, data):
        """
        This function is called whenever a new cmd_vel message is received
        """
        self.lin_cmd = data.linear.x
        self.ang_cmd = data.angular.z



    def run(self):
        """
        This is the main loop for the MCU communication node
        """
        self.mcu_startup()
        
        # Execute loop at 100 Hz: every topic published at 100/6=16.67 Hz
        rate = rospy.Rate(100)

        # Variables to save throughout the loop
        acc_x = 0
        acc_y = 0
        ang_vel_z = 0
        qx = 0
        qy = 0

        pos_x = 0
        pos_y = 0
        pos_theta = 0

        air_quality_msg = AirQuality()
        aqi_publish_count = 0

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
            rcvd = self._spi_exchange(vel_msg)
            msg = self._mcu_msg(rcvd)
            # print(msg)

            # Now do something with the received data
            if msg[0] == 7: # Received dead reckoning data
                num_unknown = 0  # Reset unknown message count

                # Extract the dead reckoning data (converts from bytes to float)
                V_dr = bytes_to_float(list(reversed(msg[1:5])))
                w_dr = bytes_to_float(list(reversed(msg[5:9])))

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

            elif msg[0] == 8:  # Recieved position data
                num_unknown = 0  # Reset unknown message count

                # Extract the position data (converts from bytes to float)
                pos_x = bytes_to_float(list(reversed(msg[1:5])))
                pos_y = bytes_to_float(list(reversed(msg[5:9])))
                pos_theta = bytes_to_float(list(reversed(msg[9:13])))

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

                self.tf_pub.publish(tf_msg)  # actually publish the data

            elif msg[0] == 9: # Received IMU data
                num_unknown = 0  # Reset unknown message count

                # Get roll/pitch in degrees
                roll = bytes_to_float(list(reversed(msg[1:5])))
                pitch = bytes_to_float(list(reversed(msg[5:9])))

                roll_rad = roll * (3.141592653589793 / 180.0)  # Convert to radians
                pitch_rad = pitch * (3.141592653589793 / 180.0)
                
                roll_pitch = quaternion_from_euler(roll_rad, pitch_rad, 0)
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

            elif msg[0] == 15:  # Received IMU Orientation XY data
                num_unknown = 0  # Reset unknown message count

                qx = bytes_to_float(list(reversed(msg[1:5])))
                qy = bytes_to_float(list(reversed(msg[5:9])))

            elif msg[0] == 16:  # Received IMU Orientation ZW data
                num_unknown = 0  # Reset unknown message count

                qz = bytes_to_float(list(reversed(msg[1:5])))
                qw = bytes_to_float(list(reversed(msg[5:9])))

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
                
                # self.imu_pub.publish(imu)  # actually publish the data
                
            elif msg[0] == 10: # Received reflective sensor data
                num_unknown = 0  # Reset unknown message count

                right_sensor = bytes_to_unsigned_int(msg[1], msg[2])
                front_sensor = bytes_to_unsigned_int(msg[3], msg[4])
                left_sensor = bytes_to_unsigned_int(msg[5], msg[6])
                
                # Actually publish the data
                self.right_sensor_pub.publish(float(right_sensor))
                self.front_sensor_pub.publish(float(front_sensor))
                self.left_sensor_pub.publish(float(left_sensor))
                button_status = msg[7]
                # Bits of button status indicates if each button is pressed
                button1_pressed = (button_status & 0b00000001) == 0b00000001
                button2_pressed = (button_status & 0b00000010) == 0b00000010
                button3_pressed = (button_status & 0b00000100) == 0b00000100
                
                if (button1_pressed != self.button_status[0]):
                    self.button_status[0] = button1_pressed
                    self.button_pub.publish(button_status)
                    if button1_pressed:
                        rospy.loginfo("[MCU Comms] Button 1 pressed")
                    else:
                        rospy.loginfo("[MCU Comms] Button 1 released")
                if (button2_pressed != self.button_status[1]):
                    self.button_status[1] = button2_pressed
                    self.button_pub.publish(button_status)
                    if button2_pressed:
                        rospy.loginfo("[MCU Comms] Button 2 pressed")
                    else:
                        rospy.loginfo("[MCU Comms] Button 2 released")
                if (button3_pressed != self.button_status[2]):
                    self.button_status[2] = button3_pressed
                    self.button_pub.publish(button_status)
                    if button3_pressed:
                        rospy.loginfo("[MCU Comms] Button 3 pressed")
                    else:
                        rospy.loginfo("[MCU Comms] Button 3 released")
            elif msg[0] == 6: # Received temperature/humidity data
                num_unknown = 0  # Reset unknown message count

                # Only publish approximately once a second
                aqi_publish_count += 1
                if aqi_publish_count >= 33:
                    temp = bytes_to_float(list(reversed(msg[1:5])))
                    humidity = bytes_to_float(list(reversed(msg[5:9])))

                    air_quality_msg.header.stamp = rospy.Time.now()
                    air_quality_msg.temperature = temp
                    air_quality_msg.relative_humidity = humidity
            elif msg[0] == 5: # Received air quality data
                num_unknown = 0  # Reset unknown message count

                if aqi_publish_count >= 33:
                    aqi_publish_count = 0

                    voc = struct.unpack('i', bytes(list(reversed(msg[1:5]))))[0]
                    nox = struct.unpack('i', bytes(list(reversed(msg[5:9]))))[0]

                    air_quality_msg.voc_index = float(voc)
                    air_quality_msg.nox_index = float(nox)
                    self.air_quality_pub.publish(air_quality_msg)
            else:
                num_unknown += 1
                # print(rcvd)

                if num_unknown >= 20:
                    rospy.loginfo("[MCU Comms] Resetting")
                    self._recover_mcu(pos_x=pos_x, pos_y=pos_y, pos_theta=pos_theta)
                    num_unknown = 0                
            rate.sleep()

    def shutdown(self):
        """
        This function is called when the node is shutdown
        """
        self._set_mcu_connected(False)
        # Send shutdown message to MCU
        shutdown_message = [90, 0b11110000,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self._spi_exchange(shutdown_message)

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
    rospy.loginfo("[MCU Comms] Shutting down MCU communication")
    comms.shutdown()

if __name__ == "__main__":
    comms = MCU_Comms()
    rospy.on_shutdown(mcu_shutdown)
    comms.run()
