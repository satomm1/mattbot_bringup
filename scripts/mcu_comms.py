import rospy
import time
import math
import spidev
import struct
import os

from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool, Float32, UInt8
from mattbot_bringup.msg import AirQuality

BAUD_RATE = 1000000 # Baud rate for SPI (coupled to MCU interrupt latency, see _spi_exchange)
SPI_SYNC = 55  # Sync byte prepended to every 16-byte payload
FRAME_LEN = 16  # Payload bytes in each direction

# Protocol v2 adds integrity checks to every 16-byte message, both directions:
#   byte 15: CRC-8 (poly 0x31, init 0xFF) over bytes 0..14
#   byte 14: (MCU -> Jetson only) sequence number, incremented for every new MCU message
# Negotiated at bringup: the Jetson sends its version in bringup byte 2, the MCU replies with its
# version in ack byte 4. Older firmware sends 0 there, and the Jetson then skips the checks.
# Byte 15 of Jetson messages is unused by older firmware, so the CRC is always sent.
PROTOCOL_VERSION = 2
SEQ_INDEX = 14
CRC_INDEX = 15

MAX_BAD_FRAMES = 20  # Consecutive unusable MCU frames (0.2 s at 100 Hz) before re-handshaking
AIR_QUALITY_PERIOD = 1.0  # Seconds between /air_quality publishes
KNOWN_MSG_TYPES = (5, 6, 7, 8, 9, 10, 15, 16)

"""
Communicates with the MCU to send and receive data via SPI.
"""

class MCU_Comms:
    """
    Sets up communication between the Jetson and the MCU
    """
    def __init__(self):

        # Initialize the node
        # Not anonymous: a second instance must replace this one, not fight it for the SPI device
        rospy.init_node('mcu_comms')

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

        # Link integrity state, set by mcu_startup()
        self.mcu_crc = False  # True when both sides speak protocol v2 (CRC + sequence checks)
        self.last_mcu_seq = None
        self.crc_errors = 0
        self.repeated_frames = 0
        self.unknown_frames = 0

        self.button_status = [False, False, False]
        # Publisher for button status
        self.button_pub = rospy.Publisher('/button_status', UInt8, queue_size=10)

        # Latched MCU connection state for DDS heartbeat bridging
        self.mcu_connected_pub = rospy.Publisher(
            "/mcu_connected", Bool, queue_size=1, latch=True
        )
        self.link_up = False
        self._set_mcu_connected(False)

        # Latest velocity command as one tuple so the SPI loop never sees a half-updated pair:
        # (linear, angular, time.monotonic() of receipt or None if no command since the last reset)
        self._cmd = (0.0, 0.0, None)
        self._timeout_logged_stamp = None

        # Send zero velocity when /cmd_vel is older than this (s). twist_mux never publishes a zero
        # when its inputs time out, so without this a dead publisher's last command runs forever.
        # <= 0 disables the timeout.
        self.cmd_vel_timeout = rospy.get_param('~cmd_vel_timeout', 0.5)

        # Subscribe to the cmd_vel topic to receive velocity commands
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)

    def _set_mcu_connected(self, connected):
        """Publish latched MCU link state (True once valid data is flowing)."""
        self.link_up = connected
        self.mcu_connected_pub.publish(Bool(data=connected))

    @staticmethod
    def _frame(payload):
        """
        Pad a Jetson message to 16 bytes and put the CRC-8 of bytes 0..14 in byte 15.
        """
        if len(payload) > CRC_INDEX:
            raise ValueError("MCU payload is limited to %d bytes, got %d" % (CRC_INDEX, len(payload)))
        frame = list(payload) + [0] * (FRAME_LEN - len(payload))
        frame[CRC_INDEX] = crc8(frame[:CRC_INDEX])
        return frame

    def _spi_exchange(self, payload):
        """
        One SPI transaction (a single CS assertion) of 18 bytes in each direction.
        Returns all 18 MISO bytes; use _mcu_msg() to get the 16-byte MCU message.

            byte:   0        1           2 .. 17           (17)
            MOSI:   sync=55  payload[0]  payload[1..15]    0x00 trailer
            MISO:   ignored  underrun    MCU msg[0..15]

        The MCU arms its TX FIFO with its 16-byte message inside the SPI RX interrupt that
        fires on the sync byte (SPI2RXHandler in JetsonSM.c). By then MISO byte 1 is already
        being clocked from an empty FIFO, so the MCU message lands in MISO bytes 2..17. The
        trailing 0x00 only clocks out the last MCU byte; the MCU discards it as a non-sync byte.

        That interrupt must also read the sync byte before MOSI byte 1 has fully arrived. If not,
        the MCU rejects the frame as "too_late": its MISO bytes are garbage and the MCU never
        processes it, so its next reply repeats the previous message. The window is one byte
        time (8 us at 1 MHz), so BAUD_RATE is tied to the MCU's interrupt latency: lowering it is
        safe, raising it can make the MCU reject every frame.

        The MCU builds each reply while handling the previous frame, so the message returned here
        answers the frame before this one.
        """
        return self.spi.xfer([SPI_SYNC] + self._frame(payload) + [0x00])

    @staticmethod
    def _mcu_msg(rcvd):
        """
        Return the 16-byte MCU message from an _spi_exchange() result: drop MISO byte 0 (clocked
        during sync) and byte 1 (FIFO underrun while the MCU interrupt loads its reply).
        """
        # print("Received from MCU:", rcvd)
        return rcvd[2:18]

    def _check_mcu_msg(self, msg):
        """
        Protocol v2: reject MCU messages with a bad CRC, or with the same sequence number as the
        last accepted one. A repeat means the MCU didn't produce a new message (it rejected or
        discarded the previous frame, or its state machine stalled), so its data is stale.
        Always True on protocol v1 firmware, which has neither field.
        """
        if not self.mcu_crc:
            return True
        if crc8(msg[:CRC_INDEX]) != msg[CRC_INDEX]:
            self.crc_errors += 1
            return False
        if msg[SEQ_INDEX] == self.last_mcu_seq:
            self.repeated_frames += 1
            return False
        self.last_mcu_seq = msg[SEQ_INDEX]
        return True

    def _recover_mcu(self, pos_x=0, pos_y=0, pos_theta=0):
        """
        Re-handshake, restoring the last pose the MCU reported. The MCU accepts bringup in every
        state (an active MCU stops its wheels first), so no shutdown message or watchdog wait
        is needed. Returns False if ROS shut down during the handshake.
        """
        self._clear_cmd()
        return self.mcu_startup(pos_x=pos_x, pos_y=pos_y, pos_theta=pos_theta)

    def mcu_startup(self, pos_x=None, pos_y=None, pos_theta=None):
        """
        Handshake with the MCU. Returns True once the confirmation is sent, False if ROS shut
        down first.

        1. Send bringup [90, 255, PROTOCOL_VERSION] every 0.1 s until the MCU acks with
           [0, 255, 0, robot_id, mcu_protocol_version].
        2. Send confirmation [90, 170, x, y, theta] to seed the MCU's dead-reckoning pose.

        Nothing acknowledges the confirmation, so /mcu_connected only goes True once run()
        receives the first valid data message.
        """
        self._set_mcu_connected(False)

        # Bringup message to MCU
        bringup_message = [90, 255, PROTOCOL_VERSION]

        # Send bringup message to MCU until received and confirmed
        wait_start = time.monotonic()
        while True:
            if rospy.is_shutdown():
                return False
            rcvd = self._spi_exchange(bringup_message)
            msg = self._mcu_msg(rcvd)
            # print(msg)

            # Check if the MCU has confirmed bringup
            if msg[0] == 0 and msg[1] == 255 and msg[2] == 0:
                mcu_version = msg[4]
                # A v2 ack carries a CRC; a corrupted one is ignored and bringup resent
                if mcu_version < 2 or crc8(msg[:CRC_INDEX]) == msg[CRC_INDEX]:
                    break
            if time.monotonic() - wait_start > 2.0:
                rospy.logwarn_throttle(5.0, "[MCU Comms] Waiting for the MCU to answer bringup (is it powered and flashed?)")
            time.sleep(0.1)

        self.robot_id = msg[3]  # Get and store the robot ID
        self.mcu_crc = min(PROTOCOL_VERSION, mcu_version) >= 2
        self.last_mcu_seq = msg[SEQ_INDEX] if self.mcu_crc else None
        self.crc_errors = self.repeated_frames = self.unknown_frames = 0
        rospy.loginfo("[MCU Comms] Robot ID: %d, MCU protocol v%d%s", self.robot_id, max(mcu_version, 1),
                      "" if self.mcu_crc else " (no CRC/sequence checks; update MCU firmware)")

        # Send confirmation message to MCU
        if pos_x is None and pos_y is None and pos_theta is None:
            confirmation_message = [90, 170]
        else:
            # Starting pose as big-endian floats (bytes 2-13)
            confirmation_message = [90, 170] + list(struct.pack('>fff', pos_x, pos_y, pos_theta))

        # Send the confirmation message to the MCU
        self._spi_exchange(confirmation_message)
        return True

    def vel_callback(self, data):
        """
        This function is called whenever a new cmd_vel message is received
        """
        lin = data.linear.x
        ang = data.angular.z
        if not (math.isfinite(lin) and math.isfinite(ang)):
            # NaN passes every range check in the MCU's SetDesiredSpeed, so never send one
            rospy.logwarn_throttle(1.0, "[MCU Comms] Ignoring non-finite cmd_vel (%s, %s); stopping", lin, ang)
            lin = ang = 0.0
        self._cmd = (lin, ang, time.monotonic())

    def _clear_cmd(self):
        """Forget the current command so nothing received before now is ever sent."""
        self._cmd = (0.0, 0.0, None)

    def _current_cmd(self):
        """Return the (linear, angular) velocity to send: zero if no command is current."""
        lin, ang, stamp = self._cmd
        if stamp is None:
            return 0.0, 0.0
        if self.cmd_vel_timeout > 0 and time.monotonic() - stamp > self.cmd_vel_timeout:
            if (lin != 0.0 or ang != 0.0) and self._timeout_logged_stamp != stamp:
                self._timeout_logged_stamp = stamp
                rospy.logwarn("[MCU Comms] No /cmd_vel for %.2f s while moving; sending zero velocity",
                              self.cmd_vel_timeout)
            return 0.0, 0.0
        return lin, ang

    @staticmethod
    def _velocity_message(lin_cmd, ang_cmd):
        """Velocity message: [45, linear (big-endian float), angular (big-endian float)]."""
        return [45] + list(struct.pack('>ff', lin_cmd, ang_cmd))  # 45 indicates velocity message

    def run(self):
        """
        This is the main loop for the MCU communication node
        """
        if not self.mcu_startup():
            return
        
        # Execute loop at 100 Hz. The MCU sends one message type per frame from a fixed schedule
        # (TxSchedule in JetsonSM.c), so each topic's rate is 100 Hz times its share of that schedule.
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
        have_temp_humidity = False
        last_air_quality_pub = 0.0

        num_bad = 0  # Consecutive frames that were corrupted, stale, or of no known type

        sensor_sequence = 0  # Sequence number for sensor messages
        while not rospy.is_shutdown():
            # Send velocity command to MCU
            rcvd = self._spi_exchange(self._velocity_message(*self._current_cmd()))
            msg = self._mcu_msg(rcvd)
            # print(msg)

            msg_type = msg[0] if self._check_mcu_msg(msg) else None
            if msg_type in KNOWN_MSG_TYPES:
                num_bad = 0
                if not self.link_up:
                    self._set_mcu_connected(True)
                    rospy.loginfo("[MCU Comms] MCU link up")

            # Now do something with the received data
            if msg_type == 7: # Received dead reckoning data

                # Extract the dead reckoning data (converts from bytes to float)
                V_dr, w_dr = struct.unpack('>ff', bytes(msg[1:9]))

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

            elif msg_type == 8:  # Recieved position data

                # Extract the position data (converts from bytes to float)
                pos_x, pos_y, pos_theta = struct.unpack('>fff', bytes(msg[1:13]))

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

            elif msg_type == 9: # Received IMU data

                # Get roll/pitch in degrees
                roll, pitch = struct.unpack('>ff', bytes(msg[1:9]))

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

            elif msg_type == 15:  # Received IMU Orientation XY data

                qx, qy = struct.unpack('>ff', bytes(msg[1:9]))

            elif msg_type == 16:  # Received IMU Orientation ZW data

                qz, qw = struct.unpack('>ff', bytes(msg[1:9]))

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
                
            elif msg_type == 10: # Received reflective sensor data

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
            elif msg_type == 6: # Received temperature/humidity data
                # Keep the latest values; they go out with the next air quality publish
                air_quality_msg.temperature, air_quality_msg.relative_humidity = struct.unpack('>ff', bytes(msg[1:9]))
                have_temp_humidity = True

            elif msg_type == 5: # Received air quality data
                # Throttle by time, not message count, so the rate doesn't depend on the MCU schedule
                now = time.monotonic()
                if have_temp_humidity and now - last_air_quality_pub >= AIR_QUALITY_PERIOD:
                    last_air_quality_pub = now

                    voc, nox = struct.unpack('>ii', bytes(msg[1:9]))

                    air_quality_msg.header.stamp = rospy.Time.now()
                    air_quality_msg.voc_index = float(voc)
                    air_quality_msg.nox_index = float(nox)
                    self.air_quality_pub.publish(air_quality_msg)
            else:
                if msg_type is not None:
                    self.unknown_frames += 1
                num_bad += 1
                # print(rcvd)

                if num_bad >= MAX_BAD_FRAMES:
                    rospy.logwarn("[MCU Comms] %d unusable MCU frames in a row (totals since handshake: "
                                  "%d CRC errors, %d repeated, %d unexpected type); re-handshaking",
                                  num_bad, self.crc_errors, self.repeated_frames, self.unknown_frames)
                    if not self._recover_mcu(pos_x=pos_x, pos_y=pos_y, pos_theta=pos_theta):
                        return
                    num_bad = 0
            rate.sleep()

    def shutdown(self):
        """
        Stop the robot and release the MCU. Called from the main thread after run() returns, so
        it never races the loop for the SPI device.

        Zero-velocity frames go first so the wheels stop even if every shutdown frame is lost
        (otherwise the MCU keeps the last speed until its 1 s JETSON_TIMEOUT). Each message is
        sent several times because a single frame can be rejected.
        """
        try:
            for message in [self._velocity_message(0.0, 0.0)] * 3 + [[90, 0b11110000]] * 3:
                self._spi_exchange(message)
                time.sleep(0.01)
        finally:
            self.spi.close()
        try:
            self._set_mcu_connected(False)
        except rospy.ROSException:
            pass  # Publishers may already be closed during ROS shutdown

def _make_crc8_table(poly=0x31):
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table

_CRC8_TABLE = _make_crc8_table()

def crc8(data):
    """
    CRC-8, poly 0x31, init 0xFF, no reflection or final XOR. Matches SpiCrc8 in JetsonSM.c
    (the same CRC the MCU uses for its Sensirion sensors).
    """
    crc = 0xFF
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc

def bytes_to_unsigned_int(high_byte, low_byte):
    # Pack the bytes into a 16 bit unsigned int
    return (high_byte << 8) | low_byte

if __name__ == "__main__":
    comms = MCU_Comms()
    try:
        comms.run()
    except rospy.ROSInterruptException:
        pass  # rate.sleep() raises this on shutdown
    finally:
        rospy.loginfo("[MCU Comms] Shutting down MCU communication")
        comms.shutdown()
