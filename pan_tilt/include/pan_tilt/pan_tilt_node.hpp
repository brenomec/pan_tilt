#include "vector"
#include <ros/ros.h>

class PanTilt
{
public:
    explicit PanTilt(ros::NodeHandle&);
    virtual ~PanTilt();
    void panCallback(const dynamixel_msgs::JointState::ConstPtr& msg);
    void tiltCallback(const dynamixel_msgs::JointState::ConstPtr& msg);
    void pan();
    void tilt();
    void cam();

private:
    ros::NodeHandle nh;
    ros::Publisher pan_pub, tilt_pub;
    ros::Subscriber pan_sub, tilt_sub;
    std::vector<float> _pan;
    std::vector<float> _tilt;
};