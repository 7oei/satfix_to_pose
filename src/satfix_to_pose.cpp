
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

bool set_initial_fix = false;
float initial_latitude,initial_longitude;
int equator_radius = 6378137;

void
SatfixCallback (const sensor_msgs::NavSatFixConstPtr& sat_fix_msg)
{
    if(sat_fix_msg->status.status == 2){
      std::cout<<"gnss : "<< sat_fix_msg->latitude << "," << sat_fix_msg->longitude << std::endl;
      float current_latitude,current_longitude;
      
      current_latitude = sat_fix_msg->latitude;
      current_longitude = sat_fix_msg->longitude;
      if(!set_initial_fix){
        initial_latitude = current_latitude;
        initial_longitude = current_longitude;
        set_initial_fix = true;
      }
      float m_per_latitude, m_per_longitude;
      m_per_latitude = (2.0 * M_PI * equator_radius) / 360;
      m_per_longitude = (2.0 * M_PI * equator_radius * cos(current_latitude / 180 * M_PI)) / 360;
      geometry_msgs::PoseStamped gnss_pose;
      gnss_pose.header.frame_id = "map";
      gnss_pose.pose.position.x = (current_longitude - initial_longitude) * m_per_longitude;
      gnss_pose.pose.position.y = (current_latitude - initial_latitude) * m_per_latitude;
      gnss_pose.pose.position.z = 0;
      gnss_pose.pose.orientation.x = 0;
      gnss_pose.pose.orientation.y = 0;
      gnss_pose.pose.orientation.z = 0;
      gnss_pose.pose.orientation.w = 0;
      pub.publish (gnss_pose);
    }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "satfix_to_pose");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/ublox/fix", 1, SatfixCallback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<geometry_msgs::PoseStamped> ("gnss_pose", 1);

  // Spin
  ros::spin ();
}