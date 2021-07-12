#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

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
	
	void vive_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void robot_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	bool pose_within_bounds(const geometry_msgs::PoseStamped& msg);
	bool calculate_pose(geometry_msgs::PoseStamped& msg_out, const geometry_msgs::PoseStamped& msg_in);
	bool pose_within_velocity_limits(const geometry_msgs::PoseStamped& msg);
};
