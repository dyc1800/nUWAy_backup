#include <iostream>
#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include "geometry_msgs/PoseStamped.h"
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

class desired_direction {
  public:
    //desired_direction();
    //~desired_direction();
    void check_feedback(const move_base_msgs::MoveBaseActionFeedbackConstPtr& msg);
    void check_goal(const geometry_msgs::PoseStamped& msg);
    int pub_direction();

  private:
    ros::NodeHandle nh;
    geometry_msgs::PoseStamped temp_pose;
    visualization_msgs::Marker marker;
    double roll, pitch, yaw_1, yaw_2, diff;
    tf::Quaternion quat;
};

void desired_direction::check_feedback(const move_base_msgs::MoveBaseActionFeedbackConstPtr& msg) {
    temp_pose.pose.position.x = (*msg).feedback.base_position.pose.position.x;
    temp_pose.pose.position.y = (*msg).feedback.base_position.pose.position.y;
    temp_pose.pose.position.z = (*msg).feedback.base_position.pose.position.z;
    tf::quaternionMsgToTF((*msg).feedback.base_position.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_1);
    return;
}

void desired_direction::check_goal(const geometry_msgs::PoseStamped& msg) {
    temp_pose.pose.orientation.x = msg.pose.orientation.x;
    temp_pose.pose.orientation.y = msg.pose.orientation.y;
    temp_pose.pose.orientation.z = msg.pose.orientation.z;
    temp_pose.pose.orientation.w = msg.pose.orientation.w;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_2);
    return;
}

int desired_direction::pub_direction() {
    ros::Subscriber feedback_sub = nh.subscribe ("move_base/feedback", 1, &desired_direction::check_feedback,this);
    ros::Subscriber goal_sub = nh.subscribe ("move_base_simple/goal", 1, &desired_direction::check_goal,this);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "desired_direction";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 2.0;
    marker.scale.y = 0.05;
    marker.scale.z = 0.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    while(ros::ok()){
	ros::spinOnce();
	sleep(0.5);
	marker.pose.position.x = temp_pose.pose.position.x;
        marker.pose.position.y = temp_pose.pose.position.y;
        marker.pose.position.z = temp_pose.pose.position.z;
        marker.pose.orientation.x = temp_pose.pose.orientation.x;
        marker.pose.orientation.y = temp_pose.pose.orientation.y;
        marker.pose.orientation.z = temp_pose.pose.orientation.z;
        marker.pose.orientation.w = temp_pose.pose.orientation.w;
	marker_pub.publish(marker);
	diff = (yaw_2 - yaw_1)*180/3.14;
	printf("%lf\n",diff);
    }
    return 0;
}

int main(int argc, char** argv) {
    ros::init (argc, argv, "desired_direction");
    desired_direction my_desired_direction;
    my_desired_direction.pub_direction();
    return 0;
}
