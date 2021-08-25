#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include <queue>

/*#include "jump_detector/filter.h"*/

class VivePoseConverterNode {
public:
	ros::NodeHandle nh;
	
	VivePoseConverterNode();
    ~VivePoseConverterNode();
	void run();

private:
	ros::Publisher pose_pub;
	ros::Subscriber vive_sub;
	ros::Subscriber robot_sub;
	ros::Subscriber stop_button_sub;
	
	double x_min;
	double x_max;
	double y_min;
	double y_max;
	double z_min;
	double z_max;
	double scale_factor;
	bool x_bounded;
	bool y_bounded;
	bool z_bounded;
	bool vive_offset_set;
	bool robot_offset_set;
	geometry_msgs::Pose vive_offset;
	geometry_msgs::Pose robot_offset;
	geometry_msgs::PoseStamped last_sent_pose;
	double max_vel;
	bool max_vel_passed;
	std::queue<geometry_msgs::PoseStamped> vive_poses;
/*	Filter filter;*/
	int max_window_length;
	bool stop_button_pressed;
	
	void vive_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void robot_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void stop_button_callback(const sensor_msgs::Joy::ConstPtr& msg);
	bool pose_within_bounds(const geometry_msgs::PoseStamped& msg);
	bool calculate_pose(geometry_msgs::PoseStamped& msg_out, const geometry_msgs::PoseStamped& msg_in);
	bool pose_within_velocity_limits(const geometry_msgs::PoseStamped& msg);
	bool vive_pose_received(const geometry_msgs::PoseStamped::ConstPtr& msg, geometry_msgs::PoseStamped& vive_pose);
};
