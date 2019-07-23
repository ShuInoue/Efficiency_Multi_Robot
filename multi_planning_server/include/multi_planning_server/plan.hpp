#ifndef _PLAN_HPP_
#define _PLAN_HPP_
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<math.h>
#include<vector>
#include<string>
#include<algorithm>
#include<iomanip>
#include<random>

struct robotData
{
    geometry_msgs::PoseStamped goal;
    nav_msgs::Odometry location;
    nav_msgs::Path path;
    double pathLength;
    int memberID;

    bool operator<(const robotData &another) const
    {
        return  pathLength < another.pathLength;
    }
};
class plan
{
    int numberOfRobots;
    int indexes;
    public:
    plan();
    double calcPathFromLocationToTarget(geometry_msgs::PoseStamped Goal, nav_msgs::Odometry Location, nav_msgs::Path path);
    double getDistance(double x, double y, double x2, double y2);
    void recursive_comb(int *indexes, int s, int rest, std::function<void(int *)> f);
    void foreach_comb(int n, int k, std::function<void(int *)> f);
};

#endif