#include<ros/ros.h>
#include<exploration_libraly/struct.h>
#include<geometry_msgs/PoseStamped.h>

double trajectryLength;

double getDistance(double x, double y, double x2, double y2)
{
    double distance=sqrt(pow(x2-x,2)+pow(y2-y,2));
    return distance;
}   

void pathToLength(const geometry_msgs::PoseStamped::ConstPtr &subOdom)
{
    static double presentOdomX=0,presentOdomY=0;
    double OdomX=0,OdomY=0;

    OdomX = subOdom->pose.position.x;
    OdomY = subOdom->pose.position.y;

    trajectryLength += getDistance(OdomX,OdomY,presentOdomX,presentOdomY);
    presentOdomX = OdomX;
    presentOdomY = OdomY;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_length");
    ros::NodeHandle nh;
    ros::Subscriber odomSub;
    ros::CallbackQueue q;
    nh.setCallbackQueue(&q);
    odomSub = nh.subscribe<geometry_msgs::PoseStamped>("/robot1/pose",1,&pathToLength);
    
    while(ros::ok())
    {
        q.callOne(ros::WallDuration(0.1));
    }
    std::cout << "all trajectry is [" << trajectryLength << "]" <<std::endl;  
    return 0;
}