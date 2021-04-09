#include <iostream>
#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

/*#define sample 300
#define angle_tol 40
#define dis_tol 0.8
#define dis_invalid 1.6
#define angle_invalid 70*/
geometry_msgs::PoseStamped goal_pose, feedback_pose;
visualization_msgs::Marker marker_arrow, marker_waypoint;
double roll, pitch, yaw_1, yaw_2;
tf::Quaternion quat;
std::vector<geometry_msgs::PoseStamped> global_path;
//nav_msgs::Path global_path;
int goal_pub_indi = 0, path_pub = 0, path_get = 0;

void get_path(const nav_msgs::Path::Ptr& msg) {
    global_path = msg->poses;
    //goal_pub_indi = 0;
    path_pub = 1;
    return;
}

void check_feedback(const move_base_msgs::MoveBaseActionFeedbackConstPtr& msg) {
    feedback_pose.pose.position.x = (*msg).feedback.base_position.pose.position.x;
    feedback_pose.pose.position.y = (*msg).feedback.base_position.pose.position.y;
    feedback_pose.pose.position.z = (*msg).feedback.base_position.pose.position.z;
    feedback_pose.pose.orientation.x = (*msg).feedback.base_position.pose.orientation.x;
    feedback_pose.pose.orientation.y = (*msg).feedback.base_position.pose.orientation.y;
    feedback_pose.pose.orientation.z = (*msg).feedback.base_position.pose.orientation.z;
    feedback_pose.pose.orientation.w = (*msg).feedback.base_position.pose.orientation.w;
    tf::quaternionMsgToTF((*msg).feedback.base_position.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_1);
    return;
}

void check_goal(const geometry_msgs::PoseStamped& msg) {
    goal_pose.pose.position.x = msg.pose.position.x;
    goal_pose.pose.position.y = msg.pose.position.y;
    goal_pose.pose.position.z = msg.pose.position.z;
    goal_pose.pose.orientation.x = msg.pose.orientation.x;
    goal_pose.pose.orientation.y = msg.pose.orientation.y;
    goal_pose.pose.orientation.z = msg.pose.orientation.z;
    goal_pose.pose.orientation.w = msg.pose.orientation.w;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_2);
    goal_pub_indi = 1;
    return;
}

double velocity_direction(const double& waypoint_x, const double& waypoint_y, const double& vehicle_x, const double& vehicle_y, const double& vehicle_heading, double& real_angle) {
    double v1_x = waypoint_x - vehicle_x;
    double v1_y = waypoint_y - vehicle_y;
    double v2_x = cos(vehicle_heading);
    double v2_y = sin(vehicle_heading);
    double dot = v1_x * v2_x + v1_y * v2_y;
    double perp_dot = v1_x * v2_y - v1_y * v2_x;
    double vector_angle = atan2(perp_dot, dot);
    if (vector_angle > M_PI/2) {
	real_angle = -(M_PI - vector_angle)*180/M_PI;
        return -1;
    }
    if (vector_angle < -M_PI/2) {
	real_angle = (M_PI + vector_angle)*180/M_PI;
        return -1;
    }
    real_angle = vector_angle*180/M_PI;
    return 1;
}

int main(int argc, char** argv) {
    ros::init (argc, argv, "desired_direction_new");
    ros::NodeHandle nh;
    ros::Subscriber feedback_sub = nh.subscribe ("move_base/feedback", 1, check_feedback);
    ros::Subscriber goal_sub = nh.subscribe ("move_base_simple/goal", 1, check_goal);
    ros::Subscriber path_sub = nh.subscribe ("move_base_node/PoseFollower/global_plan", 1, get_path);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    ros::Publisher angle_pub = nh.advertise<geometry_msgs::Twist>("steering_angle", 1);
    geometry_msgs::Twist steering_angle;
    int path_length, segment_length, sample;
    if (nh.getParam("sample", sample))
    {
      ROS_INFO("Got param: %lf\n", sample);
    }
    else
    {
      ROS_ERROR("Failed to get param 'sample'");
    }
    double yaw_3[sample-1], dis_diff, angle_diff, desired_speed, desired_speed_test, angle_coe, angle_tol, dis_tol, dis_invalid, angle_invalid, sleep_time;

    if (nh.getParam("sleep_time", sleep_time))
    {
      ROS_INFO("Got param: %lf\n", sleep_time);
    }
    else
    {
      ROS_ERROR("Failed to get param 'sleep_time'");
    }
    if (nh.getParam("angle_tol", angle_tol))
    {
      ROS_INFO("Got param: %lf\n", angle_tol);
    }
    else
    {
      ROS_ERROR("Failed to get param 'angle_tol'");
    }
    if (nh.getParam("dis_tol", dis_tol))
    {
      ROS_INFO("Got param: %lf\n", dis_tol);
    }
    else
    {
      ROS_ERROR("Failed to get param 'dis_tol'");
    }
    if (nh.getParam("dis_invalid", dis_invalid))
    {
      ROS_INFO("Got param: %lf\n", dis_invalid);
    }
    else
    {
      ROS_ERROR("Failed to get param 'dis_invalid'");
    }
    if (nh.getParam("angle_invalid", angle_invalid))
    {
      ROS_INFO("Got param: %lf\n", angle_invalid);
    }
    else
    {
      ROS_ERROR("Failed to get param 'angle_invalid'");
    }

    if (nh.getParam("desired_speed", desired_speed))
    {
      ROS_INFO("Got param: %lf\n", desired_speed);
    }
    else
    {
      ROS_ERROR("Failed to get param 'desired_speed'");
    }
    if (nh.getParam("angle_coe", angle_coe))
    {
      ROS_INFO("Got param: %lf\n", angle_coe);
    }
    else
    {
      ROS_ERROR("Failed to get param 'angle_coe'");
    }
    geometry_msgs::PoseStamped waypoints[sample];
    marker_arrow.header.frame_id = "map";
    marker_arrow.header.stamp = ros::Time::now();
    marker_arrow.ns = "desired_direction_new";
    marker_arrow.id = 0;
    marker_arrow.type = visualization_msgs::Marker::ARROW;
    //marker.action = visualization_msgs::Marker::ADD;
    marker_arrow.scale.x = 12.0;
    marker_arrow.scale.y = 0.3;
    marker_arrow.scale.z = 0.0;
    marker_arrow.color.r = 0.0f;
    marker_arrow.color.g = 1.0f;
    marker_arrow.color.b = 0.0f;
    marker_arrow.color.a = 1.0;
    marker_arrow.lifetime = ros::Duration();

    marker_waypoint.header.frame_id = "map";
    marker_waypoint.header.stamp = ros::Time::now();
    marker_waypoint.ns = "waypoint";
    marker_waypoint.id = 0;
    marker_waypoint.type = visualization_msgs::Marker::SPHERE;
    //marker.action = visualization_msgs::Marker::ADD;
    marker_waypoint.scale.x = 0.6;
    marker_waypoint.scale.y = 0.6;
    marker_waypoint.scale.z = 0.0;
    marker_waypoint.color.r = 1.0f;
    marker_waypoint.color.g = 0.0f;
    marker_waypoint.color.b = 0.0f;
    marker_waypoint.color.a = 1.0;
    marker_waypoint.lifetime = ros::Duration();
    while(ros::ok()){
	steering_angle.angular.z = 0;
	steering_angle.linear.x = 0;
	angle_pub.publish(steering_angle);
	ros::spinOnce();
	//sleep(0.5);
	/*if(goal_pub_indi == 1) {
	    while(goal_pub_indi == 1) {
		ros::spinOnce();
	    }
	}*/
	if(path_pub == 1) {
	    path_length = global_path.size();
	    //printf("%lf\n",sample);
	    //segment_length = path_length/sample;
	    waypoints[sample-1] = goal_pose;
	    for(int i = 0; i < sample-1; i++) {
		segment_length = path_length*(i+1)/sample;
		waypoints[i] = global_path[segment_length];
		tf::quaternionMsgToTF(waypoints[i].pose.orientation, quat);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw_3[i]);
	    }
	    path_pub = 0;
	    path_get = 1;
	    goal_pub_indi = 0;
	}
	if(path_get == 1 && path_pub == 0 && goal_pub_indi == 0) {
	    marker_arrow.action = visualization_msgs::Marker::ADD;
	    marker_waypoint.action = visualization_msgs::Marker::ADD;
	    for(int i = 0; i < sample && path_get == 1 && path_pub == 0 && goal_pub_indi == 0 && ros::ok(); i++) {
		marker_waypoint.pose.position.x = waypoints[i].pose.position.x;
        	marker_waypoint.pose.position.y = waypoints[i].pose.position.y;
        	marker_waypoint.pose.position.z = waypoints[i].pose.position.z;
        	marker_waypoint.pose.orientation.x = waypoints[i].pose.orientation.x;
        	marker_waypoint.pose.orientation.y = waypoints[i].pose.orientation.y;
        	marker_waypoint.pose.orientation.z = waypoints[i].pose.orientation.z;
        	marker_waypoint.pose.orientation.w = waypoints[i].pose.orientation.w;
		marker_pub.publish(marker_waypoint);
		if(i < sample-1) {    
		    do {
			ros::spinOnce();
			angle_diff = (yaw_3[i] - yaw_1)*180/3.14;
			dis_diff = sqrt((waypoints[i].pose.position.x - feedback_pose.pose.position.x)*(waypoints[i].pose.position.x - feedback_pose.pose.position.x) + 			(waypoints[i].pose.position.y - feedback_pose.pose.position.y)*(waypoints[i].pose.position.y - feedback_pose.pose.position.y));
			marker_arrow.pose.position.x = feedback_pose.pose.position.x;
        		marker_arrow.pose.position.y = feedback_pose.pose.position.y;
        		marker_arrow.pose.position.z = feedback_pose.pose.position.z;
        		marker_arrow.pose.orientation.x = waypoints[i].pose.orientation.x;
        		marker_arrow.pose.orientation.y = waypoints[i].pose.orientation.y;
        		marker_arrow.pose.orientation.z = waypoints[i].pose.orientation.z;
        		marker_arrow.pose.orientation.w = waypoints[i].pose.orientation.w;
			marker_pub.publish(marker_arrow);
			if(angle_diff < -180)
			    angle_diff = angle_diff + 360;
			if(angle_diff > 180)
			    angle_diff = angle_diff - 360;
			desired_speed_test = velocity_direction(waypoints[i].pose.position.x, waypoints[i].pose.position.y, feedback_pose.pose.position.x, feedback_pose.pose.position.y, yaw_1, angle_diff)*desired_speed;
			if(angle_diff > angle_invalid || angle_diff < -angle_invalid || dis_diff > dis_invalid) {
			    //goal_pub.publish(goal_pose);
			    printf("large angle:%lf\n",angle_diff);
			    steering_angle.angular.z = 0;
			    steering_angle.linear.x = 0;
			    angle_pub.publish(steering_angle);
			    goal_pub.publish(goal_pose);
			} else {
			    printf("%lf\n",angle_diff);
			    steering_angle.angular.z = angle_diff*angle_coe/190;
			    steering_angle.linear.x = desired_speed_test;
			    angle_pub.publish(steering_angle);
			}
			sleep(sleep_time);
		    } while(ros::ok() && path_get == 1 && path_pub == 0 && goal_pub_indi == 0 && !(dis_diff < dis_tol && angle_diff <angle_tol && angle_diff >-angle_tol));
		} else {
		    do {
			ros::spinOnce();
			angle_diff = (yaw_2 - yaw_1)*180/3.14;
			dis_diff = sqrt((waypoints[i].pose.position.x - feedback_pose.pose.position.x)*(waypoints[i].pose.position.x - feedback_pose.pose.position.x) + 			(waypoints[i].pose.position.y - feedback_pose.pose.position.y)*(waypoints[i].pose.position.y - feedback_pose.pose.position.y));
			marker_arrow.pose.position.x = feedback_pose.pose.position.x;
        		marker_arrow.pose.position.y = feedback_pose.pose.position.y;
        		marker_arrow.pose.position.z = feedback_pose.pose.position.z;
        		marker_arrow.pose.orientation.x = waypoints[i].pose.orientation.x;
        		marker_arrow.pose.orientation.y = waypoints[i].pose.orientation.y;
        		marker_arrow.pose.orientation.z = waypoints[i].pose.orientation.z;
        		marker_arrow.pose.orientation.w = waypoints[i].pose.orientation.w;
			marker_pub.publish(marker_arrow);
			if(angle_diff < -180)
			    angle_diff = angle_diff + 360;
			if(angle_diff > 180)
			    angle_diff = angle_diff - 360;
			desired_speed_test = velocity_direction(waypoints[i].pose.position.x, waypoints[i].pose.position.y, feedback_pose.pose.position.x, feedback_pose.pose.position.y, yaw_1, angle_diff)*desired_speed;
			if(angle_diff > angle_invalid || angle_diff < -angle_invalid || dis_diff > dis_invalid) {
			    //goal_pub.publish(goal_pose);
			    printf("large angle:%lf\n",angle_diff);
			    steering_angle.angular.z = 0;
			    steering_angle.linear.x = 0;
			    angle_pub.publish(steering_angle);
			    goal_pub.publish(goal_pose);
			} else {
			    printf("%lf\n",angle_diff);
			    steering_angle.angular.z = angle_diff*angle_coe/190;
			    steering_angle.linear.x = desired_speed_test;
			    angle_pub.publish(steering_angle);
			}
			sleep(sleep_time);
		    } while(ros::ok() && path_get == 1 && path_pub == 0 && goal_pub_indi == 0 && !(dis_diff < dis_tol/3 && angle_diff <angle_tol && angle_diff >-angle_tol));
		}
	    }
	    marker_arrow.action = visualization_msgs::Marker::DELETE;
	    marker_waypoint.action = visualization_msgs::Marker::DELETE;
	    marker_pub.publish(marker_arrow);
	    marker_pub.publish(marker_waypoint);
	    path_get = 0;
	}
    }
    return 0;
}
