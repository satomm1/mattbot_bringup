import rospy
from geometry_msgs.msg import Pose
import tf
import rospkg

import numpy as np
import os

class GetKnownPoints:

    def __init__(self):
        rospy.init_node('get_known_points', anonymous=True)
        self.known_points = {}
        self.trans_listener = tf.TransformListener()

    def run(self):
        while not rospy.is_shutdown():
            user_input = input("Enter a number to store the robot's current location or 'q' to quit: ")
            if user_input.lower() == 'q':
                break
            try:
                point_number = int(user_input)
                (translation, rotation) = self.trans_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
                x = translation[0]
                y = translation[1]
                self.known_points[point_number] = (x, y)
                rospy.loginfo(f"Stored point {point_number} at x: {x}, y: {y}")
            except ValueError:
                rospy.logwarn("Please enter a valid number.")
            except rospy.ROSException as e:
                rospy.logerr(f"Error getting robot pose: {e}")

        if len(self.known_points) >= 4:

            # Check that points are numbered 1, 2, 3, ...
            point_numbers = np.array(list(self.known_points.keys()))
            point_numbers = np.sort(point_numbers)

            if point_numbers[0] != 1 or point_numbers[-1] != len(point_numbers):
                rospy.logwarn("Please store points numbered 1, 2, 3, ...")
                return
            

            # Find the mattbot_dds package and write the known points to the known_points.txt file
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('mattbot_dds')
            known_points_file = os.path.join(package_path, 'scripts/known_points.txt')

            # Write the x,y values of the known points to the file. x and y are separated by a comma. Each point is on a new line.
            with open(known_points_file, 'w') as f:
                for i in range(len(self.known_points)):
                    point_number = i + 1
                    x, y = self.known_points[point_number]
                    # Round to 2 decimal places
                    x = round(x, 2)
                    y = round(y, 2)
                    f.write(f"{x},{y}\n")
                    print(f"Stored point {point_number} at x: {x}, y: {y}")

            rospy.loginfo(f"Stored {len(self.known_points)} known points in {known_points_file}")
            if len(self.known_points) < 6:
                rospy.logwarn("6-10 known points spread across the whole map give the best TPS map alignment.")
        else:
            rospy.logwarn("Please store at least 4 known points (the TPS map transform needs >= 4; 6-10 is best). "
                          "Spread them across the map, avoid collinear points, and use the same order on every robot.")

if __name__ == '__main__':
    KnownPoints = GetKnownPoints()
    KnownPoints.run()