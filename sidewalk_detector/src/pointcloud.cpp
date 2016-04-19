// ROS core
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl_visualization/pcl_visualizer.h>
//openCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
// cv_bridge
#include <cv_bridge/cv_bridge.h>

ros::Publisher pub_in;
ros::Publisher pub_out;

void cloud_detect (const sensor_msgs::PointCloud2ConstPtr& input){
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
        usleep (10000);
        viewer->spinOnce (10);
        // if no new messages appear, just stop processing
        if (!cloud_msg || cloud_msg == old_cloud_msg)
            return;
        old_cloud_msg = cloud_msg;
        
        // get message from ROS and convert to point cloud
        PointCloud<PointXYZRGB> point_cloud;
        fromROSMsg(*cloud_msg, point_cloud);

        // if the sensor point cloud provides "far range data", use it to compute the sensor_pose   
        PointCloud<PointWithViewpoint> far_ranges;
        RangeImage::extractFarRanges(*cloud_msg, far_ranges);
        if (pcl::getFieldIndex(*cloud_msg, "vp_x")>=0) {
            PointCloud<PointWithViewpoint> tmp_pc;
            fromROSMsg(*cloud_msg, tmp_pc);
            Eigen::Vector3f average_viewpoint = RangeImage::getAverageViewPoint(tmp_pc);
            sensor_pose = Eigen::Translation3f(average_viewpoint) * sensor_pose;
        }
                
        PointCloud<PointXYZRGB>::Ptr point_cloud_ptr (new PointCloud<PointXYZRGB>(point_cloud));
        PointCloud<PointXYZRGB>::Ptr point_cloudfilt_ptr (new PointCloud<PointXYZRGB>); // filtered pc

        // Filter clouds in Z dimension (min, max)
        FilterPointCloudZ(point_cloud_ptr, output, 0.0f, 10.0f);

        //publish filtered point cloud
        pub.publish((*output));
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "sidewalk_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/color/depth/points", 1000, cloud_detect);

  // Create a ROS publisher for the output point cloud
  pub_in = nh.advertise<sensor_msgs::PointCloud2> ("/sidewalk_detector/depth/points_in", 1);
  pub_out = nh.advertise<sensor_msgs::PointCloud2> ("/sidewalk_detector/depth/points_out", 1);

  // Spin
  ros::spin ();
}

