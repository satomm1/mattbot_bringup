import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import xml.etree.ElementTree as ET


class TransformPublisher:

    def __init__(self):

        # Initialize the ROS node
        rospy.init_node("transform_publisher", anonymous=True)
        
        robot_description = rospy.get_param("~robot_description", "../urdf/robot_tf.urdf")

        if not robot_description:
            rospy.logerr("Robot description parameter not found.")
            return

        tree = ET.parse(robot_description)
        root = tree.getroot()

        self.robot_name = root.get("name", "unknown_robot")

        # Get all links and joints from the URDF
        self.links = root.findall("link")
        self.joints = root.findall("joint")

        # Create a transform broadcaster
        self.tf_broadcaster = rospy.Publisher("/tf", TFMessage, queue_size=10)

    def publish_transform(self, transform, parent_link, child_link, x, y, z):

        # Create a TransformStamped message
        transform_msg = TransformStamped()
        transform_msg.header.frame_id = parent_link
        transform_msg.child_frame_id = child_link
        transform_msg.transform.translation.x = float(x)
        transform_msg.transform.translation.y = float(y)
        transform_msg.transform.translation.z = float(z)
        transform_msg.transform.rotation.x = 0.0
        transform_msg.transform.rotation.y = 0.0
        transform_msg.transform.rotation.z = 0.0    
        transform_msg.transform.rotation.w = 1.0
        transform_msg.header.stamp = rospy.Time.now()
        # Add the transform message to the TFMessage
        transform.transforms.append(transform_msg)

        return transform
        
    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            transform = TFMessage()
            for joint in self.joints:
                joint_name = joint.get("name")
                parent_link = joint.find("parent").get("link")
                child_link = joint.find("child").get("link")
                x,y,z = joint.find("origin").get("xyz", "0 0 0").split()

                transform = self.publish_transform(transform, parent_link, child_link, x, y, z)

            # Publish the transform message
            self.tf_broadcaster.publish(transform)
            rate.sleep()

if __name__ == "__main__":

    transform_publisher = TransformPublisher()
    transform_publisher.run()