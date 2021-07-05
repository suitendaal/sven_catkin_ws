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
	
	double x_min;
	double x_max;
	double y_min;
	double y_max;
	double z_min;
	double z_max;
	bool x_bounded;
	bool y_bounded;
	bool z_bounded;
	
	void vive_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	bool pose_within_bounds(const geometry_msgs::PoseStamped& msg);
};
