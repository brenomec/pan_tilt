#include "std_msgs/Float64.h"
#include "opencv2/opencv.hpp"
#include "dynamixel_msgs/JointState.h"
#include "sstream"
#include "pan_tilt/pan_tilt_node.hpp"
#include <math.h> 

using namespace cv;

bool takePic = false;
bool started = false;
bool moveP = true;
bool moveT = true;
std_msgs::Float64 goalPan;
std_msgs::Float64 goalTilt;
std_msgs::Float64 posePan;
std_msgs::Float64 poseTilt;
int pic = 0;
int act = 0;
int focus = 0;
int retry = 0;

PanTilt::PanTilt(ros::NodeHandle& n) : 
    nh(n),
    _pan ({0.0,4.7,3.15,1.55,0.0}),
    _tilt ({-1.0,0.0,1.0})
{          
        pan_pub = nh.advertise<std_msgs::Float64>("/joint3_controller/command", 1);
        tilt_pub = nh.advertise<std_msgs::Float64>("/joint4_controller/command", 1);

        pan_sub = nh.subscribe("/joint3_controller/state", 10, &PanTilt::panCallback,this);
        tilt_sub = nh.subscribe("/joint4_controller/state", 10, &PanTilt::tiltCallback,this);
    }

PanTilt::~PanTilt() {

}

void PanTilt::panCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
    goalPan.data = msg->current_pos;
}
    
void PanTilt::tiltCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
{
    goalTilt.data = msg->current_pos;
}
    
void PanTilt::pan() {
                if(!takePic && act == 0 && moveP && started){
                    posePan.data = _pan.back();
                    ROS_INFO("pan moving");
                    pan_pub.publish(posePan);
                    while(fabs(posePan.data-goalPan.data)>0.1 && ros::ok()){
                    ros::spinOnce();
                    retry++;
                    if(retry>1000){
                        ROS_INFO("trying again %d",retry);
                        pan_pub.publish(posePan);
                        ros::Duration(1).sleep();
                    }
                    }
                    retry = 0;
                    ROS_INFO("pan moved");
                    _pan.pop_back();
                    moveP = false;
                    //3.15,4.7,0,1.55
                    }
}
    
void PanTilt::tilt(){
                    if(!takePic && moveT && started){
                    poseTilt.data = _tilt.at(act);
                    ROS_INFO("tilt moving");
                    tilt_pub.publish(poseTilt);
                    while(fabs(poseTilt.data-goalTilt.data)>0.1 && ros::ok()){
                    ros::spinOnce();
                    retry++;
                    if(retry>1000){
                        ROS_INFO("trying again %d",retry);
                        tilt_pub.publish(poseTilt);
                        ros::Duration(1).sleep();
                    }
                    }
                    retry = 0;
                    ROS_INFO("tilt moved");
                    takePic = true;
                    moveT = false;
                    //-1,0,1
                    }
}
    
void PanTilt::cam(){
                    takePic=false;
                    moveP = true;
                    moveT = true;
                    focus = 0;
                    ROS_INFO("cam %d",pic);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pan_tilt");
    ros::NodeHandle nh;    

    VideoCapture cap(0);
    if(!cap.isOpened()) 
        return -1;

    Mat frame(1024,1024,CV_8UC3);
    namedWindow("cam",1);
    PanTilt PTObject(nh);
    while(ros::ok() && pic < 12)
    {
        ros::spinOnce();
        act = pic%3;
        PTObject.pan();
        PTObject.tilt();
        Mat frame(1024,1024,CV_8UC3);
        cap >> frame;
        imshow("cam", frame);
        if(takePic){
            focus++;
            ROS_INFO("focus count %d", focus);
            if(focus>9){
            std::ostringstream dir;
            dir<<"/home/breno/Cam/"<<(pic ++)<<".png";
            std::string filename = dir.str();
            dir.str("");
            imwrite(filename, frame); 
            PTObject.cam();
            }
        }
        started = true;
        if(waitKey(30) >= 0) break;
    }
    ros::spinOnce();
    act = pic%3;
    PTObject.pan();
    act = 1;
    PTObject.tilt();
    ros::shutdown();
    return 0;

}