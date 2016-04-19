#!/usr/bin/python

# Extract images from a bag file and export to given directory

# Start up ROS pieces.
PKG = 'sidewalk_detector'
import roslib; roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
import os
import sys

class ImageCreator():

    image_type = ".jpg"
    #desired_topic = "/camera/color/image_raw"
    index_in_filename = True
    index_format = "04d"
    image_index = 0
    bagfilename = "/home/administrator/catkin_ws/src/sidewalk_detector/src/realsense_coding_challenge_1.bag"
    save_dir = "/home/administrator/catkin_ws/src/sidewalk_detector/src/images"

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Get parameters when starting node from a launch file.
        if len(sys.argv) < 1:
            desired_topic = rospy.get_param('desired_topic')
            #filename = rospy.get_param('~filename')
            #rospy.loginfo("Bag filename = %s", self.bagfilename)
            rospy.loginfo("Topic name = %s", desired_topic)
        # Get parameters as arguments to 'rosrun my_package bag_to_images.py <save_dir> <filename>', where save_dir and filename exist relative to this executable file.
        else:
            desired_topic = os.path.join(sys.path[0], sys.argv[1])
            #filename = os.path.join(sys.path[0], sys.argv[2])
            rospy.loginfo("Topic name = %s", desired_topic)

        # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
        self.bridge = CvBridge()

        # Open bag file.
        with rosbag.Bag(self.bagfilename, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                topic_parts = topic.split('/')
                # first part is empty string
                #if len(topic_parts) == 3 and topic_parts[2] == self.desired_topic:
                if topic == desired_topic:
		# remove all files from directory
		    save_path = str(self.save_dir) +"/"+ str(topic_parts[1]) + "/" + str(topic_parts[3])
                    #for the_file in os.listdir(save_path):
			#file_path = os.path.join(save_path, the_file)
			#try:
                	#	if os.path.isfile(file_path):
			#		os.unline(file_path)
                  	#except Exception as e:
			#	print(e)

		    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError, e:
                        print e
                    timestr = "%.3f" % msg.header.stamp.to_sec()
                    if self.index_in_filename:
                        image_name = str(save_path) +  "/" + format(self.image_index, self.index_format) + self.image_type
                    else:
                        image_name = str(save_path) + "/" + topic_parts[1] + "-" + timestr + self.image_type
                    cv2.imwrite(image_name, cv2.flip(cv_image,0))
                    self.image_index = self.image_index + 1
			
# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node(PKG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException: pass
